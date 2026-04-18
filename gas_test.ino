const int MQ135_AO = A0;
const int MQ135_DO = 2;

const float VREF = 5.0;
const int ADC_MAX = 1023;

int baseline = 0;
bool baselineSet = false;

int readMQ135Average(int samples = 10) {
  long total = 0;
  for (int i = 0; i < samples; i++) {
    total += analogRead(MQ135_AO);
    delay(10);
  }
  return total / samples;
}

String airChangeLabel(int delta) {
  if (delta < -30) return "lower_than_baseline";
  if (delta < 30) return "near_baseline";
  if (delta < 80) return "slightly_elevated";
  if (delta < 150) return "elevated";
  return "highly_elevated";
}

void setup() {
  Serial.begin(9600);
  pinMode(MQ135_DO, INPUT);

  delay(5000);  // short warmup for testing
  baseline = readMQ135Average(30);
  baselineSet = true;

  Serial.print("BASELINE:");
  Serial.println(baseline);
}

void loop() {
  int rawAnalog = readMQ135Average(10);
  int digitalState = digitalRead(MQ135_DO);
  float voltage = rawAnalog * (VREF / ADC_MAX);

  int delta = rawAnalog - baseline;

  Serial.print("MQ135_RAW:");
  Serial.print(rawAnalog);

  Serial.print(",BASELINE:");
  Serial.print(baseline);

  Serial.print(",DELTA:");
  Serial.print(delta);

  Serial.print(",VOLTAGE:");
  Serial.print(voltage, 3);

  Serial.print(",DO:");
  Serial.print(digitalState);

  Serial.print(",TREND:");
  Serial.println(airChangeLabel(delta));

  delay(500);
}