char response[128];

void setup() {
  Serial.begin(9600, SERIAL_8N1);
  Serial.print("READY\n");
}

void loop() {
  static int index = 0;
  if (Serial.available() > 0) {
    char buff = Serial.read();
    response[index] = buff;
    index++;
    if (response[index-1] == '\n' || response[index-1] == '\r') {
      response[index] = '\0';
      index = 0;
      Serial.print(response);
    }
  }
}
