# DryboxSensor
Battery powered filament drybox humidity sensor

Humidity and temperature is detected by ATH10 sensor. Data is displayed on SSD1306 OLED display. TP4056 module is used to charge 18650 battery cell. Additionally some buzzer and button are needed. I used Arudino Pro Mini, but any 3.3V level Arduino board should be fine.

Sensor most of the time works in low power mode. If hight humidity or low power voltage is detected, then alarm with buzzer is triggered. 