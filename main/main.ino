// include header files
#include <Arduino.h>
#include <LiquidCrystal.h>
#include <dac.h> 
#include "Elevator.h"
#include "DC_motor.h"
#include "Stepper.h"
#include "LCD.h"
#include "PhysicalState.h"
#include "PID.h"
#include "Control.h"
#include "LED.h"

// Class names
Elevator elev;
Control control;
DCmotor Dcmoto;
LCD lcden;
Stepper step;

// Global variabel
float currentEncoderFloor = 0.0;
int currentLogicalFloor = 0;
long physicalTargetFloor = 0;
PhysicalState physicalState = P_IDLE;
unsigned long doorTimer = 0;
extern const unsigned long doorOpenTime = 2000; // 2 sekunder


void setup() {
  Serial.begin(115200);
  while (!Serial); 
  Serial.println("Initialiserer system...");
  
  //Initialize Stepper (Doors)
  Stepper::Step_init();

  //Initialize DC Motor (Lift)
  DCmotor::DC_init();

  //Initialize Encoders
  DCmotor::Enc_init();
  
  //Initialize DAC
  dac_init();
  set_dac(3500, 3500);

  //Initialize Buttons (UI)
  control.Button_init();

  //Initialize LED
  LED::LED_init();

  //Initialize LCD
  LCD::Lcd_init();

  //Initialize Timers
  PID::lastTime = millis();
  delay(1000); // Wait 1 sec
  Serial.println("Klar!");
}

void loop() {
 
  // Check for button presses
  control.checkButtons();

  // Check for seriell input 
  control.checkSerialInput();
   // Read inputs
  Dcmoto.GetEncoderPos();

  // Prosses state machine
  elev.processPhysicalState();

  // Updatde LCD
  lcden.updateIfNeeded(elev, physicalState);
}