#include <dac.h>

// Eksempel: Laboppgave 7 - Stepmotor (Servolab)
// Forutsetning: DAC_bibliotek finnes og gir dac_init() / set_dac(msb, lsb)


// Pins (i henhold til dokumentet)
const int A_enable = 69; // D69
const int B_enable = 67; // D67
const int A_phase  = 68; // D68
const int B_phase  = 66; // D66

const int potPin = A0; // potensiometer for hastighet

void setup() {
  // init DAC og sett maks til 1 A (ifølge oppgaven)
  dac_init();
  set_dac(4095, 4095);   // 12-bit max på begge kanaler = ~1 A (oppgitt)

  // pinMode for styringspinner
  pinMode(A_enable, OUTPUT);
  pinMode(B_enable, OUTPUT);
  pinMode(A_phase, OUTPUT);
  pinMode(B_phase, OUTPUT);

  pinMode(potPin, INPUT);
}

/////////////////////
// Full-step sekvens (4 trinn)
// Tabellen i dokumentet (forenklet):
// Step 1: A_en=1 B_en=1 A_ph=0 B_ph=0
// Step 2: A_en=1 B_en=1 A_ph=0 B_ph=1
// Step 3: A_en=1 B_en=1 A_ph=1 B_ph=1
// Step 4: A_en=1 B_en=1 A_ph=1 B_ph=0
/////////////////////
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

void fullStepBackward() {
  // motsatt rekkefølge
  digitalWrite(A_enable, HIGH); digitalWrite(B_enable, HIGH);
  digitalWrite(A_phase, HIGH); digitalWrite(B_phase, LOW);
  pulseDelay();

  digitalWrite(A_phase, HIGH); digitalWrite(B_phase, HIGH);
  pulseDelay();

  digitalWrite(A_phase, LOW);  digitalWrite(B_phase, HIGH);
  pulseDelay();

  digitalWrite(A_phase, LOW);  digitalWrite(B_phase, LOW);
  pulseDelay();
}

/////////////////////
// Half-step (8 trinn) - jevnere gange
// Rekkefølge følger tabellen i dokumentet (halvstepsekvens)
/////////////////////
void halfStepForward() {
  digitalWrite(A_enable, HIGH); digitalWrite(B_enable, HIGH);

  // 1
  digitalWrite(A_phase, LOW); digitalWrite(B_phase, LOW); pulseDelay();
  // 2
  digitalWrite(A_phase, LOW); digitalWrite(B_phase, HIGH); digitalWrite(B_enable, LOW); pulseDelay(); // X i tabellen = ignorere en pin eller toggles
  // 3
  digitalWrite(A_enable, HIGH); digitalWrite(B_enable, HIGH); digitalWrite(A_phase, HIGH); digitalWrite(B_phase, LOW); pulseDelay();
  // 4
  digitalWrite(A_enable, HIGH); digitalWrite(B_enable, LOW); digitalWrite(A_phase, HIGH); pulseDelay();
  // 5
  digitalWrite(A_enable, HIGH); digitalWrite(B_enable, HIGH); digitalWrite(A_phase, HIGH); digitalWrite(B_phase, HIGH); pulseDelay();
  // 6
  digitalWrite(A_enable, LOW); digitalWrite(B_enable, HIGH); digitalWrite(B_phase, HIGH); pulseDelay();
  // 7
  digitalWrite(A_enable, HIGH); digitalWrite(B_enable, HIGH); digitalWrite(A_phase, LOW); digitalWrite(B_phase, HIGH); pulseDelay();
  // 8
  digitalWrite(A_enable, HIGH); digitalWrite(B_enable, LOW); digitalWrite(A_phase, LOW); pulseDelay();
}

void halfStepBackward() {
  // Implementer motsatt rekkefølge (for oversiktlighet, kall forward i en loop med -1)
  // (For enkelhet kan du lage en array med 8 kombinasjoner og iterere baklengs.)
}

/////////////////////
// Delay/hastighetsfunksjon
// Bruk potensiometer (A0) og map direkte til delayMicroseconds slik oppgaven sier.
/////////////////////
void pulseDelay() {
  // les pot og sett delay. Formel i oppgave:
  // delayMicroseconds( analogRead(A0)/4 + 1 );
  int r = analogRead(potPin);
  unsigned int dt = r / 4 + 1; // 0..1023 -> 1..256 us (juster etter ønsket område)
  delayMicroseconds(dt);
}

/////////////////////
// En enkel funksjon for å flytte N fullsteps (posisjonskontroll)
/////////////////////
void moveFullSteps(long nSteps, bool forward) {
  for (long i=0; i<nSteps; ++i) {
    if (forward) fullStepForward();
    else         fullStepBackward();
  }
}

void loop() {
  // Demonstrasjon: ett step per sekund (som i oppgaven) - bruk pot for hastighet:
  fullStepForward();        // ett step (fire innvendige trinn kjøres)
  delay(1000);              // til testing. Bytt ut med hastighetsstyring når du vil.
  // For real hastighetskontroll, kalle fullStepForward() uten delay(1000) og la pulseDelay styre.
}
