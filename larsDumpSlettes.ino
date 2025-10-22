#include <Arduino.h>
#include <LiquidCrystal.h>
#include <dac.h>
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// LCD setup
LiquidCrystal lcd(41, 40, 37, 36, 35, 34);
const int LCD_BACKLIGHT = 4;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Motor pins
const int A_enable = 69;
const int B_enable = 67;
const int A_phase  = 68;
const int B_phase  = 66;
const int potPin = A0;
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Timing
unsigned long motorStartTime = 0;
const unsigned long MOTOR_RUN_TIME = 5000; // 5 seconds
//%%%%%%%%%%%%%%%%%%%%%%%%    MOTOR   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void pulseDelay() {
  int r =420; //0-1023   justere frat men må være over 300 jaffal
  unsigned int dt = map(r, 0, 1023, 1000, 10000);
  delayMicroseconds(dt);
}

void fullStepForward() {
  digitalWrite(A_enable, HIGH); digitalWrite(B_enable, HIGH);

  digitalWrite(A_phase, LOW);  digitalWrite(B_phase, LOW);
  pulseDelay();

  digitalWrite(A_phase, LOW);  digitalWrite(B_phase, HIGH);
  pulseDelay();

  digitalWrite(A_phase, HIGH); digitalWrite(B_phase, HIGH);
  pulseDelay();

  digitalWrite(A_phase, HIGH); digitalWrite(B_phase, LOW);
  pulseDelay();
}

void stopMotor() {
  digitalWrite(A_enable, LOW);
  digitalWrite(B_enable, LOW);
}
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
void setup() {
  pinMode(LCD_BACKLIGHT, OUTPUT);
  digitalWrite(LCD_BACKLIGHT, HIGH);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Door closed");
  lcd.setCursor(0, 1);
  lcd.print("Going upp");

  dac_init();
  set_dac(4095, 4095);

  pinMode(A_enable, OUTPUT);
  pinMode(B_enable, OUTPUT);
  pinMode(A_phase, OUTPUT);
  pinMode(B_phase, OUTPUT);
  pinMode(potPin, INPUT);

  motorStartTime = millis();
}

void loop() {
  if (millis() - motorStartTime < MOTOR_RUN_TIME) {
    fullStepForward();
  } else {
    stopMotor();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Motor stopped");
    lcd.setCursor(0, 1);
    lcd.print("Open doors");
    // Prevent repeated LCD updates
    while (true) { delay(1000); }
  }
}





//Søppeldynga

/*
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


// === All in One Servo Lab: DC Motor Control ===
// Driver: TI DRV8840
// Mode: Unipolar, Low Decay (brake when PWM=0)

#define MOTOR_PHASE 6    // Direction control
#define MOTOR_ENABLE 7   // PWM control
#define MOTOR_DECAY 5    // Decay mode (low = brake, high = coast)
#define MOTOR_CURRENT A3 // Current sensor input

int pwmValue = 0;        // PWM duty cycle (0-255)
bool direction = true;   // Motor direction

void setup() {
  pinMode(MOTOR_PHASE, OUTPUT);
  pinMode(MOTOR_ENABLE, OUTPUT);
  pinMode(MOTOR_DECAY, OUTPUT);
  pinMode(MOTOR_CURRENT, INPUT);

  // Set decay to low (brake mode)
  digitalWrite(MOTOR_DECAY, LOW);

  // Increase PWM frequency on Timer4 (pin 7 → OC4D)
  // 0x01 gives ~32 kHz (from 490 Hz default)
  TCCR4B = TCCR4B & 0b11111000 | 0x01;

  Serial.begin(9600);
  Serial.println("ServoLab DC Motor Test - Unipolar Low Decay");
}

void loop() {
  // Sweep speed up and down
  for (pwmValue = 0; pwmValue <= 255; pwmValue += 5) {
    driveMotor(direction, pwmValue);
    delay(20);
  }
  for (pwmValue = 255; pwmValue >= 0; pwmValue -= 5) {
    driveMotor(direction, pwmValue);
    delay(20);
  }

  // Reverse direction
  direction = !direction;
  delay(1000);
}

void driveMotor(bool dir, int pwm) {
  digitalWrite(MOTOR_PHASE, dir ? HIGH : LOW);
  analogWrite(MOTOR_ENABLE, pwm);

  // Optional: read motor current
  int raw = analogRead(MOTOR_CURRENT);
  float current = (raw - 512) * 0.03789;  // A = (V-2.5V)*37.89
  Serial.print("Dir: "); Serial.print(dir ? "FWD" : "REV");
  Serial.print(" | PWM: "); Serial.print(pwm);
  Serial.print(" | Current: "); Serial.print(current);
  Serial.println(" A");
}



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
*/

