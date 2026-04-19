#include <Wire.h>
#include <MPU6050_light.h>
#include <FastLED.h>

MPU6050 mpu(Wire);
unsigned long lastPrint = 0;

#define LED_PIN 3
#define NUM_LEDS 30
#define BRIGHTNESS 50

CRGB leds[NUM_LEDS];
String incomingLine = "";

void showSafe() {
  fill_solid(leds, NUM_LEDS, CRGB::Green);
  FastLED.show();
}

void showDanger() {
  fill_solid(leds, NUM_LEDS, CRGB::Red);
  FastLED.show();
}

void showHelper() {
  fill_solid(leds, NUM_LEDS, CRGB::Blue);
  FastLED.show();
}

void showOff() {
  fill_solid(leds, NUM_LEDS, CRGB::Black);
  FastLED.show();
}

void handleCommand(String cmd) {
  cmd.trim();

  if (cmd == "SAFE") {
    showSafe();
  } else if (cmd == "DANGER") {
    showDanger();
  } else if (cmd == "HELPER") {
    showHelper();
  } else if (cmd == "OFF") {
    showOff();
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n') {
      handleCommand(incomingLine);
      incomingLine = "";
    } else if (c != '\r') {
      incomingLine += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  showOff();

  byte status = mpu.begin();
  if (status != 0) {
    while (1);
  }

  delay(1000);
  mpu.calcOffsets();
}

void loop() {
  mpu.update();
  readSerialCommands();

  if (millis() - lastPrint >= 100) {
    lastPrint = millis();

    Serial.print(mpu.getAccX(), 6);   Serial.print(",");
    Serial.print(mpu.getAccY(), 6);   Serial.print(",");
    Serial.print(mpu.getAccZ(), 6);   Serial.print(",");
    Serial.print(mpu.getGyroX(), 6);  Serial.print(",");
    Serial.print(mpu.getGyroY(), 6);  Serial.print(",");
    Serial.print(mpu.getGyroZ(), 6);  Serial.print(",");
    Serial.print(mpu.getAngleX(), 6); Serial.print(",");
    Serial.print(mpu.getAngleY(), 6); Serial.print(",");
    Serial.println(mpu.getAngleZ(), 6);
  }
}