#include <LiquidCrystal.h>

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

const int gasPin = A1;
const int buzzerPin = 10;

String line1 = "Booting...";
String line2 = "Waiting...";
String buzzerMode = "OFF";

unsigned long lastBuzzToggle = 0;
bool buzzState = false;
String incomingLine = "";

void drawLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1.substring(0, 16));
  lcd.setCursor(0, 1);
  lcd.print(line2.substring(0, 16));
}

void updateBuzzer() {
  unsigned long now = millis();

  if (buzzerMode == "OFF") {
    noTone(buzzerPin);
    return;
  }

  if (buzzerMode == "DANGER") {
    if (now - lastBuzzToggle > 500) {
      lastBuzzToggle = now;
      buzzState = !buzzState;
      if (buzzState) tone(buzzerPin, 1200);
      else noTone(buzzerPin);
    }
    return;
  }

  if (buzzerMode == "NEAR") {
    if (now - lastBuzzToggle > 180) {
      lastBuzzToggle = now;
      buzzState = !buzzState;
      if (buzzState) tone(buzzerPin, 1800);
      else noTone(buzzerPin);
    }
    return;
  }

  if (buzzerMode == "HELP") {
    if (now - lastBuzzToggle > 900) {
      lastBuzzToggle = now;
      tone(buzzerPin, 900, 120);
    }
    return;
  }
}

void handleCommand(String msg) {
  int l1Start = msg.indexOf("L1:");
  int l2Start = msg.indexOf("|L2:");
  int bzStart = msg.indexOf("|BZ:");

  if (l1Start >= 0 && l2Start >= 0 && bzStart >= 0) {
    line1 = msg.substring(l1Start + 3, l2Start);
    line2 = msg.substring(l2Start + 4, bzStart);
    buzzerMode = msg.substring(bzStart + 4);

    while (line1.length() < 16) line1 += " ";
    while (line2.length() < 16) line2 += " ";

    drawLCD();
  }
}

void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);
  pinMode(buzzerPin, OUTPUT);
  drawLCD();
}

void loop() {
  int gasValue = analogRead(gasPin);

  // send gas reading to Python
  Serial.print("GAS:");
  Serial.println(gasValue);

  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n') {
      handleCommand(incomingLine);
      incomingLine = "";
    } else if (c != '\r') {
      incomingLine += c;
    }
  }

  updateBuzzer();
  delay(100);
}