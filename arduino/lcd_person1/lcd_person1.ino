#include <LiquidCrystal.h>

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

const int gasPin = A1;
const int buzzer = 10; //buzzer to arduino pin 9


String line1 = "Booting...";
String line2 = "Waiting...";

void drawLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1.substring(0, 16));
  lcd.setCursor(0, 1);
  lcd.print(line2.substring(0, 16));
}

void setup() {
  Serial.begin(115200);
  lcd.begin(16, 2);
  drawLCD();
  delay(2000);
  pinMode(buzzer, OUTPUT); // Set buzzer - pin 9 as an output

}

void loop() {
  int gasValue = analogRead(gasPin);

  Serial.print("Gas Level: ");
  Serial.println(gasValue);

  if (gasValue > 300) {
    line1 = "GAS ALERT!";
    line2 = "HIGH LEVEL";
    tone(buzzer, 1000); // Send 1KHz sound signal...
    delay(1000);         // ...for 1 sec
    noTone(buzzer);     // Stop sound...
    delay(1000);         // ...for 1sec
  } else {
    line1 = "Gas Level:";
    line2 = String(gasValue);
  }

  while (line1.length() < 16) line1 += " ";
  while (line2.length() < 16) line2 += " ";

  drawLCD();

  delay(500);


}
