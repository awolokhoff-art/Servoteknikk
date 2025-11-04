#ifndef LCD_H
#define LCD_H

#include <Arduino.h>
#include <LiquidCrystal.h>
#include "PhysicalState.h" // Inkluder enum-definisjonen
#include "Elevator.h"      // Inkluder hele Elevator-klassen for å bruke Elevator::State

// --- LCD Variables ---
extern LiquidCrystal lcd;
extern const int LCD_BACKLIGHT;
extern unsigned long lastLcdUpdate;
extern const long lcdUpdateInterval_Active;
extern const long lcdUpdateInterval_Idle;

// --- Heisvariabler brukt i LCD ---
extern int currentLogicalFloor;
extern long physicalTargetFloor;
extern PhysicalState physicalState;
extern Elevator elev;

// --- Funksjoner som konverterer til tekst ---
const char* getLogicalStateString(Elevator::State s);
const char* getPhysicalStateString(PhysicalState s);

// --- Funksjoner ---
void updateLcd();
void Lcd_init();

#endif
