#include <Arduino.h>
#include <LiquidCrystal.h>
#include <dac.h>

// LCD setup
LiquidCrystal lcd(41, 40, 37, 36, 35, 34);
const int LCD_BACKLIGHT = 4;

// Motor pins
const int A_enable = 69;
const int B_enable = 67;
const int A_phase  = 68;
const int B_phase  = 66;
const int potPin = A0;

// Timing
unsigned long motorStartTime = 0;
const unsigned long MOTOR_RUN_TIME = 5000; // 5 seconds

void pulseDelay() {
  int r = analogRead(potPin);
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

void setup() {
  pinMode(LCD_BACKLIGHT, OUTPUT);
  digitalWrite(LCD_BACKLIGHT, HIGH);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Door open");
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
    lcd.print("5 sec elapsed");
    // Prevent repeated LCD updates
    while (true) { delay(1000); }
  }
}
