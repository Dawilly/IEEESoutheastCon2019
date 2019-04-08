#define BUFFER_SIZE 128
#define STATUS_PIN 36

char response[BUFFER_SIZE];

void setup() {
  Serial.begin(9600, SERIAL_8N1);
  pinMode(STATUS_PIN, OUTPUT);
  digitalWrite(STATUS_PIN, HIGH);
  Serial.print("READY\n");
}

void loop() {
  static int index = 0;
  while (Serial.available() > 0) {
    delay(50);
    digitalWrite(STATUS_PIN, LOW);
    char buff = Serial.read();
    response[index] = toupper(buff);
    index++;
    if (index == BUFFER_SIZE || response[index-1] == '\n'
        || response[index-1] == '\r') {
      response[index] = '\0';
      index = 0;
      Serial.print(response);
    }
  }
  digitalWrite(STATUS_PIN, HIGH);
}
