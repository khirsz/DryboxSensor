#include <Adafruit_AHTX0.h>
#include <Adafruit_SSD1306.h>
#include <LowPower.h>

/// Battery definitions ///

static constexpr auto INTERNAL_VREF_MV = 1100L;  // Needs to be calibrated

static constexpr auto BAT_MAX_MV = 3800L;
static constexpr auto BAT_MIN_MV = 3000L;
static constexpr auto BAT_ALARM_MV = 3100L;

static constexpr auto BAT_LEVELS = 4;

/// Misc definitions ///

static constexpr auto DISPLAY_OFF_TIME = 10000UL;
static constexpr auto HELLO_TIME = 2000UL;
static constexpr auto HUMIDITY_ALARM_LEVEL = 30.0f;

//#define DEBUG

/// Pin definitions ///

// Button pin needs low pass filter for debuncing (100nF + 1KOhm), because it is used as an interrupt trigger
static constexpr auto BUTTON_PIN = 2;
static constexpr auto BUZZER_PIN = 3;

/// Sensor definitions ///

Adafruit_AHTX0 aht;

/// OLED definitions ///

static constexpr auto SCREEN_WIDTH = 128;  // OLED display width, in pixels
static constexpr auto SCREEN_HEIGHT = 64;  // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
static constexpr auto OLED_RESET = -1;  // Reset pin # (-1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/// State machine definitions ///
enum class SensorState : uint8_t { displayInit = 0,
                                   displayOn,
                                   lowPower,
                                   alarmCheck,
                                   alarmOn,
                                   alarmOnBlink };

enum class AlarmType : uint8_t { noAlarm = 0,
                                 humidityAlarm,
                                 batAlarm };

volatile static bool buzzerOff = false;
volatile SensorState state = SensorState::displayInit;

struct DisplayData {
  float humidity;
  float temp;
  long voltage;
  bool hideHumidity;
  bool hideBat;
};

/// Funcions ///

void displayError(const char* msg) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 25);
  display.print(F("Error: "));
  display.print(msg);
  display.display();
}

void displayLogo() {
  display.setCursor(9, 25);
  display.print(F("DryBox Sensor V1.0"));
  display.display();
  delay(HELLO_TIME);
  display.clearDisplay();
}

void displayTitle() {
  display.setTextSize(1);
  display.setCursor(27, 0);
  display.print(F("Drybox Sensor"));
}

void displayHumidity(float humidity, bool hideValue = false) {
  display.setTextSize(2);
  display.setCursor(5, 18);
  display.print(F("Hum: "));

  if (!hideValue) {
    display.print(humidity, 1);
    display.print("%");
  }
}

void displayTemp(float temp) {
  display.setTextSize(1);
  display.setCursor(0, 48);
  display.print(F("T: "));
  display.print(temp, 1);
  display.print("C");
}

void displayBat(int voltage, bool hideValue = false) {
  display.setTextSize(1);
  display.setCursor(62, 48);
  display.print(F("Bat: "));

  if (!hideValue) {
    display.print("[");
    auto level = calculateBatLevel(voltage);
    for (auto i = 0; i < BAT_LEVELS; i++) {
      if (i <= level) {
        display.print("#");
      } else {
        display.print(" ");
      }
    }
    display.print("]");
  }
}

void displayMainScreen(const DisplayData& displayData) {
  display.clearDisplay();
  displayTitle();
  displayHumidity(displayData.humidity, displayData.hideHumidity);
  displayTemp(displayData.temp);
  displayBat(displayData.voltage, displayData.hideBat);
  display.display();
}

int calculateBatLevel(long voltage) {
  int level = map(voltage, BAT_MIN_MV, BAT_MAX_MV, 0, BAT_LEVELS);
  return (level < BAT_LEVELS) ? level : (BAT_LEVELS - 1);
}

void initAlarm() {
  display.ssd1306_command(SSD1306_DISPLAYON);
  buzzerOff = false;
  state = SensorState::alarmOn;
}

void buttonInterrupt() {
  if (state >= SensorState::alarmOn) {
    buzzerOff = true;
#ifdef DEBUG
    Serial.println(F("ISR: Buzzer off"));
#endif
  } else {
    state = SensorState::displayInit;
#ifdef DEBUG
    Serial.println(F("ISR: Display init"));
#endif
  }
}

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);             // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);  // Convert
  while (bit_is_set(ADCSRA, ADSC)) {};
  result = ADCL;
  result |= ADCH << 8;
  result = INTERNAL_VREF_MV * 1024L / result;  // Back-calculate AVcc in mV
  return result;
}

/// Setup ///

void setup() {
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println(F("DryBox Humidity sensor"));
#endif

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonInterrupt, FALLING);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);

  displayLogo();

  if (!aht.begin()) {
#ifdef DEBUG
    Serial.println(F("Could not find AHT? Check wiring"));
#endif
    displayError("AHT not found!");
    while (1) { delay(500); };
  }
#ifdef DEBUG
  Serial.println(F("AHT10 or AHT20 found"));
#endif
}

/// Main loop ///

void loop() {
  static unsigned long displayOnTimeStamp;
  static AlarmType alarmType = AlarmType::noAlarm;

  auto vcc = readVcc();
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);  // populate temp and humidity objects with fresh data

  DisplayData displayData{ humidity.relative_humidity, temp.temperature, vcc, false, false };

#ifdef DEBUG
  /*Serial.print(F("Temperature: "));
  Serial.print(temp.temperature);
  Serial.println(F(" degrees C"));
  Serial.print(F("Humidity: "));
  Serial.print(humidity.relative_humidity);
  Serial.println(F("% rH"));*/

  Serial.print(F("State: "));
  Serial.println(static_cast<uint8_t>(state));
#endif

  switch (state) {
    case SensorState::displayInit:
      display.ssd1306_command(SSD1306_DISPLAYON);
      displayOnTimeStamp = millis();
      state = SensorState::displayOn;
      break;

    case SensorState::displayOn:
      displayMainScreen(displayData);
      if (millis() - displayOnTimeStamp > DISPLAY_OFF_TIME) {
        display.ssd1306_command(SSD1306_DISPLAYOFF);
        state = SensorState::lowPower;
      }
      break;

    case SensorState::lowPower:
      state = SensorState::alarmCheck;
      LowPower.powerStandby(SLEEP_8S, ADC_OFF, BOD_OFF);
      break;

    case SensorState::alarmCheck:
      if (humidity.relative_humidity > HUMIDITY_ALARM_LEVEL) {
        initAlarm();
        alarmType = AlarmType::humidityAlarm;
      } else if (vcc < BAT_ALARM_MV) {
        initAlarm();
        alarmType = AlarmType::batAlarm;
      } else {
        state = SensorState::lowPower;
      }
      break;

    case SensorState::alarmOn:
      displayMainScreen(displayData);
      if (!buzzerOff) {
        digitalWrite(BUZZER_PIN, HIGH);
      }
      state = SensorState::alarmOnBlink;
      break;

    case SensorState::alarmOnBlink:
      displayData.hideHumidity = (alarmType == AlarmType::humidityAlarm) ? true : false;
      displayData.hideBat = (alarmType == AlarmType::batAlarm) ? true : false;
      displayMainScreen(displayData);
      digitalWrite(BUZZER_PIN, LOW);
      state = SensorState::alarmOn;
      break;
  }

#ifdef DEBUG
  Serial.print(F("Vcc value: "));
  Serial.print(vcc);
  Serial.println(F("mV"));

  Serial.print(F("Bat level: "));
  Serial.println(calculateBatLevel(vcc));
#endif

  delay(1000);
}
