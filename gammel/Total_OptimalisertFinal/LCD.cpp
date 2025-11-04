#include "LCD.h"
#include "Elevator.h"

// --- Initialisering av LCD ---
LiquidCrystal lcd(41, 40, 37, 36, 35, 34);
const int LCD_BACKLIGHT = 4;
unsigned long lastLcdUpdate = 0;
const long lcdUpdateInterval_Active = 250;  // 4 Hz
const long lcdUpdateInterval_Idle   = 5000; // 0.2 Hz

// --- Funksjoner som konverterer enum til tekst ---
const char* getPhysicalStateString(PhysicalState s) {
    switch (s) {
        case P_IDLE: return "CLSD";
        case P_MOVING: return "CLSD";
        case P_OPENING_DOOR: return "HLF";
        case P_DOOR_OPEN: return "OPN";
        case P_CLOSING_DOOR: return "HLF";
        default: return "???";
    }
}

const char* getLogicalStateString(Elevator::State s) {
    switch (s) {
        case Elevator::State::Idle: return "Ide";
        case Elevator::State::MovingUp: return "Up ";
        case Elevator::State::MovingDown: return "Dwn";
        default: return "???";
    }
}

// --- Oppdater LCD ---
void updateLcd() {
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

// --- Initialisering av LCD ---
void Lcd_init() {
    pinMode(LCD_BACKLIGHT, OUTPUT);
    digitalWrite(LCD_BACKLIGHT, HIGH);
    lcd.begin(16, 2);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Elevator ready");
    lcd.setCursor(0, 1);
    lcd.print("Floors: 0-7");
}
