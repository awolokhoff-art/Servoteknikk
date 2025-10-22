const int led1 = 44;
const int led2 = 45;
const int led3 = 46;

void setup() {
  Serial.begin(9600);           // Start serial communication
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  Serial.println("Press 1, 2, or 3 to toggle LEDs.");
}

void loop() {
  if (Serial.available() > 0) {       // Check if a key is pressed
    char key = Serial.read();          // Read the key

    switch (key) {
      case '1':
        digitalWrite(led1, !digitalRead(led1));  // Toggle LED1
        break;
      case '2':
        digitalWrite(led2, !digitalRead(led2));  // Toggle LED2
        break;
      case '3':
        digitalWrite(led3, !digitalRead(led3));  // Toggle LED3
        break;
      default:
        Serial.println("Press 1, 2, or 3 only.");
        break;
    }
  }
}
