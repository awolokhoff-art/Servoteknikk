#include "LCD.h"
#include "Elevator.h" // State hasAnyStops
extern Elevator elev;

// initilizing the LCD
LiquidCrystal LCD::lcd(41, 40, 37, 36, 35, 34);
const int LCD::LCD_BACKLIGHT = 4;

unsigned long LCD::lastLcdUpdate = 0;

const long LCD::lcdUpdateInterval_Active = 250;  // 4 Hz
const long LCD::lcdUpdateInterval_Idle   = 5000; // 0.2 Hz

int LCD::currentLogicalFloor = 0;

long LCD::physicalTargetFloor = 0;

PhysicalState LCD::physicalState;

const char* LCD::getLogicalStateString(Elevator::State s) {
    switch (s) {
        case Elevator::State::Idle: return "Ide";
        case Elevator::State::MovingUp: return "Up ";
        case Elevator::State::MovingDown: return "Dwn";
        default: return "???";
    }
}

const char* LCD::getPhysicalStateString(PhysicalState s) {
    switch (s) {
        case P_IDLE: return "CLSD";
        case P_MOVING: return "CLSD";
        case P_OPENING_DOOR: return "HLF";
        case P_DOOR_OPEN: return "OPN";
        case P_CLOSING_DOOR: return "HLF";
        default: return "???";
    }
}

void LCD::updateLcd() {
    lcd.clear();
    
    // Linje 0: Etasjeinfo
    lcd.setCursor(0, 0);
    lcd.print("Floor:");
    lcd.print(currentLogicalFloor);
    lcd.print("|Target:");
    lcd.print(physicalTargetFloor);

    // Linje 1: Tilstandsinfo
    lcd.setCursor(0, 1);
    lcd.print("Sta:");
    lcd.print(getLogicalStateString(elev.state));
    lcd.print("|Door:");
    lcd.print(getPhysicalStateString(physicalState));
}

void LCD::Lcd_init() {
    pinMode(LCD_BACKLIGHT, OUTPUT);
    digitalWrite(LCD_BACKLIGHT, HIGH);
    lcd.begin(16, 2);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Elevator ready");
    lcd.setCursor(0, 1);
    lcd.print("Floors: 0-7");
}

void LCD::updateIfNeeded(const Elevator& elev,PhysicalState physicalState)
{

  //Bestem riktig oppdateringsintervall
  long currentInterval;
  
  if (physicalState == P_IDLE && elev.state == Elevator::State::Idle && !elev.hasAnyStops()) {
    currentInterval = LCD::lcdUpdateInterval_Idle;
  } else {
    // Systemet er aktivt (flytter seg, dører åpne, eller har jobber som venter)
    currentInterval = LCD::lcdUpdateInterval_Active;
  }
  // Sjekk mot det valgte intervallet
  if (millis() - LCD::lastLcdUpdate > currentInterval) {
    LCD::updateLcd();
    LCD::lastLcdUpdate = millis();
  }
  delay(5);
}
