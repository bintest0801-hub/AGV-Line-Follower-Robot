int adcPins[8] = {1,2,3,10,11,12,13,14};

int values[8];
int state[8];

int threshold = 2000;

void setup() {
  Serial.begin(115200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
}

int readADC(int pin) {
  int sum = 0;
  for (int i = 0; i < 4; i++) {
    sum += analogRead(pin);
  }
  return sum >> 2; // chia 4
}

void loop() {

  // đọc tất cả kênh
  for (int i = 0; i < 8; i++) {
    values[i] = readADC(adcPins[i]);
    state[i] = (values[i] < threshold) ? 1 : 0;
  }

  // in ADC
  Serial.print("ADC: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(values[i]);
    Serial.print("\t");
  }

  // in trạng thái
  Serial.print("| BIN: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(state[i]);
    Serial.print(" ");
  }

  // visual line
  Serial.print("| ");
  for (int i = 0; i < 8; i++) {
    Serial.print(state[i] ? "█" : "_");
  }

  Serial.println();

  delay(10);
}
