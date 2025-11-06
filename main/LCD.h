#ifndef LCD_H
#define LCD_H

#include <Arduino.h>
#include <LiquidCrystal.h>
#include "PhysicalState.h" 
#include "Elevator.h"

class LCD{
private:
  //LCD Variables
  static LiquidCrystal lcd;

  static const int LCD_BACKLIGHT;
 
  static const long lcdUpdateInterval_Active;
  static const long lcdUpdateInterval_Idle;

  // Konverts text to State and PhysicalState
  static const char* getLogicalStateString(Elevator::State s);
  static const char* getPhysicalStateString(PhysicalState s);
  
  
public:
  static PhysicalState physicalState;

  //Elevator variabels used in LCD
  static int currentLogicalFloor;

  static long physicalTargetFloor;
  static unsigned long lastLcdUpdate;

  static void updateLcd();
  static void Lcd_init();
  static void updateIfNeeded(const Elevator& elev, PhysicalState physicalState);
 

 
};
#endif
