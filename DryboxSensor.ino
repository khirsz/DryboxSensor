#include <Adafruit_AHTX0.h>
#include <Adafruit_SSD1306.h>
#include <LowPower.h>

/// Battery definitions ///

#define INTERNAL_VREF_MV 1100L  // Needs to be calibrated

#define BAT_MAX_MV 3800
#define BAT_MIN_MV 3000
#define BAT_ALARM_MV 3100

/// Misc definitions ///

#define DISPLAY_OFF_TIME 10000
#define HELLO_TIME 2000
#define HUMIDITY_ALARM_LEVEL 30.0

#define DEBUG

/// Pin definitions ///

// Button pin needs low pass filter for debuncing (100nF + 1KOhm), because it is used as an interrupt trigger
#define BUTTON_PIN 2
#define BUZZER_PIN 3
#define BAT_PIN A0

/// Sensor definitions ///

Adafruit_AHTX0 aht;

/// OLED definitions ///

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1  // Reset pin # (-1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/// State machine definitions ///
enum class SensorState : uint8_t { displayInit = 0,
                                   displayOn,
                                   lowPower,
                                   alarmCheck,
                                   alarmOn,
                                   alarmOnBlink };

volatile static bool buzzerOff = false;
volatile SensorState state = SensorState::displayInit;

/// Funcions ///

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

void displayHumidity(const sensors_event_t& humidity, bool hideValue = false) {
  display.setTextSize(2);
  display.setCursor(5, 18);
  display.print(F("Hum: "));

  if (!hideValue) {
    display.print(humidity.relative_humidity, 1);
    display.print(F("%"));
  }
}

void displayTemp(const sensors_event_t& temp) {
  display.setTextSize(2);
  display.setCursor(5, 48);
  display.print(F("Tmp: "));
  display.print(temp.temperature, 1);
  display.print(F("C"));
}

void displayMainScreen(const sensors_event_t& humidity, const sensors_event_t& temp, bool hideValue = false) {
  display.clearDisplay();
  displayTitle();
  displayHumidity(humidity, hideValue);
  displayTemp(temp);
  display.display();
}

void buttonInterrupt() {
  if (state >= SensorState::alarmOn) {
    buzzerOff = true;
#ifdef DEBUG
    Serial.println("ISR: Buzzer off");
#endif
  } else {
    state = SensorState::displayInit;
#ifdef DEBUG
    Serial.println("ISR: Display init");
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
  Serial.begin(115200);
  Serial.println("DryBox Humidity sensor");

  if (!aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
    while (1) delay(500);
  }
  Serial.println("AHT10 or AHT20 found");

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonInterrupt, FALLING);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(WHITE);

  displayLogo();
}

/// Main loop ///

void loop() {
  static unsigned long displayOnTimeStamp;

  auto vcc = readVcc();
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);  // populate temp and humidity objects with fresh data
#ifdef DEBUG
  /*Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degrees C");
  Serial.print("Humidity: ");
  Serial.print(humidity.relative_humidity);
  Serial.println("% rH");*/

  Serial.print("State: ");
  Serial.println(static_cast<uint8_t>(state));
#endif

  switch (state) {
    case SensorState::displayInit:
      display.ssd1306_command(SSD1306_DISPLAYON);
      displayOnTimeStamp = millis();
      state = SensorState::displayOn;
      break;

    case SensorState::displayOn:
      displayMainScreen(humidity, temp);
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
        display.ssd1306_command(SSD1306_DISPLAYON);
        buzzerOff = false;
        state = SensorState::alarmOn;
      } else {
        state = SensorState::lowPower;
      }
      break;

    case SensorState::alarmOn:
      displayMainScreen(humidity, temp);
      if (!buzzerOff) {
        digitalWrite(BUZZER_PIN, HIGH);
      }
      state = SensorState::alarmOnBlink;
      break;

    case SensorState::alarmOnBlink:
      displayMainScreen(humidity, temp, true);
      digitalWrite(BUZZER_PIN, LOW);
      state = SensorState::alarmOn;
      break;
  }

#ifdef DEBUG
  Serial.print("Vcc value: ");
  Serial.print(vcc);
  Serial.println("mV");

  Serial.println(map(vcc, BAT_MIN_MV, BAT_MAX_MV, 0, 4) * 25);
#endif

  delay(1000);
}
