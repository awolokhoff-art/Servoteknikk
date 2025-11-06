#include "Elevator.h"
#include "LCD.h" // physicalState updateLcd currentLogicalFloor physicalTargetFloor physicalState lastLcdUpdate
#include "DC_motor.h" //PulsesPerRevolution EncoderCount PWMpin PwmValue  directionPin MotorDirection  floorTolerance
#include "PID.h" //integral lastError lastTime computePID
#include "Stepper.h" // OpenDoor CloseDoor
#include <Arduino.h>
#include "LED.h" //

extern PhysicalState physicalState;

extern int currentLogicalFloor;

extern float currentEncoderFloor;

extern long physicalTargetFloor;
extern unsigned long doorTimer;
extern const unsigned long doorOpenTime; // 2 sekunder


Elevator::Elevator(State state_, int current)
    : currentFloor(current), state(state_), lowerFloor(0), upperFloor(MAX_FLOOR)
{
    clearStops();
}
//Finds the *next* physical floor to stop at based on the current *logical* direction from the Elevator class.
int Elevator::findNextTargetInDirection() {
  if (state == Elevator::State::MovingUp) {
    // 1. Find closest stop (car or hall-up) in direction of travel
    for (int f = currentLogicalFloor + 1; f <= upperFloor; f++) {
      if (stops[f] || upStops[f]) return f;
    }
    // 2. If none, find highest hall-down call (turnaround point)
    for (int f = upperFloor; f > currentLogicalFloor; f--) {
      if (downStops[f]) return f;
    }

    for(int f = currentLogicalFloor -1; f >= lowerFloor; f--){
      if(stops[f]) return f;
    }
  }
  
  else if (state == Elevator::State::MovingDown) {
    // 1. Find closest stop (car or hall-down) in direction of travel
    for (int f = currentLogicalFloor - 1; f >= lowerFloor; f--) {
      if (stops[f] || downStops[f]) return f;
    }
    // 2. If none, find lowest hall-up call (turnaround point)
    for (int f = lowerFloor; f < currentLogicalFloor; f++) {
      if (upStops[f]) return f;
    }
    for (int f = currentLogicalFloor +1; f<= upperFloor; f++)
    if(stops[f]) return f;
  }
  
  return -1; // No target found
}

bool Elevator::hasStopsUpFromAny(int floor) const {
    for (int i = floor + 1; i <= upperFloor; ++i) if (upStops[i] || downStops[i] || stops[i]) return true;
    return false;
}

bool Elevator::hasStopsDownFromAny(int floor) const {
    for (int i = floor - 1; i >= lowerFloor; --i) if (upStops[i] || downStops[i] || stops[i]) return true;
    return false;
}

bool Elevator::acceptedFloorRange(int floor) const {
    return floor >= lowerFloor && floor <= upperFloor;
}

bool Elevator::hasAnyStops() const {
    for (int i = 0; i < NUM_FLOORS; ++i)
        if (stops[i] || upStops[i] || downStops[i]) return true;
    return false;
}

bool Elevator::addCarRequest(int floor) {
    if (!acceptedFloorRange(floor)) return false;
    if (floor == currentFloor && state == State::Idle) return false;

    stops[floor] = true;
    startElevatorMoving(floor);
    return true;
}

bool Elevator::addHallRequest(int floor, Request request) {
    if (!acceptedFloorRange(floor)) return false;

  
    if (floor == currentFloor && state == State::Idle) return false; 
    if (floor == upperFloor && request == Request::Up) return false;
    if (floor == lowerFloor && request == Request::Down) return false;
    
    if (request == Request::Up) {
        upStops[floor] = true;
    }
    if (request == Request::Down) {
        downStops[floor] = true;
    }

    startElevatorMoving(floor); 
    return true;
}
void Elevator::clearStops() {
    for (int i = 0; i < NUM_FLOORS; ++i) {
        stops[i] = upStops[i] = downStops[i] = false;
    }
}

void Elevator::setStop(int floor) {
    if (acceptedFloorRange(floor)) stops[floor] = true;
}
//Clears all requests for a given floor and turns off its LED.
void Elevator::clearRequestsAt(int floor) {
  if (floor < 0 || floor > MAX_FLOOR) return;
  
  // Check if any request exists before printing
  if (stops[floor] || upStops[floor] || downStops[floor]) {
    Serial.print("Phys: Servicing floor ");
    Serial.println(floor);
    stops[floor] = false;
    upStops[floor] = false;
    downStops[floor] = false;
    digitalWrite(LED::ledPins[floor], LOW);
  }
}

void Elevator::changeElevatorMoving() {
    if (!hasAnyStops()) {
        state = State::Idle;
        return;
    }

    if (state == State::MovingUp) {
        bool hasRequestsAbove = hasStopsUpFromAny(currentFloor);
        bool hasRequestsBelow = hasStopsDownFromAny(currentFloor);

        if (!hasRequestsAbove && hasRequestsBelow){
          state = State::MovingDown;
        }
       
    }

    if (state == State::MovingDown) {

        bool hasRequestsBelow = hasStopsDownFromAny(currentFloor);
        bool hasRequestsAbove = hasStopsUpFromAny(currentFloor);
        
         if (!hasRequestsBelow && hasRequestsAbove) {
            state = State::MovingUp;
        }
     
        return;
    }

    if (state == State::Idle) {
        if (hasStopsUpFromAny(currentFloor))
            state = State::MovingUp;
        else if (hasStopsDownFromAny(currentFloor))
            state = State::MovingDown;
    }
}

void Elevator::startElevatorMoving(int floor) {
    if (state == State::Idle) {
        if (currentFloor < floor) state = State::MovingUp;
        else if (currentFloor > floor) state = State::MovingDown;
    }
}

void Elevator::processPhysicalState() {
  
  //State machine
  switch (physicalState) {
    Serial.print("DEBUG: this ->state = ");
    Serial.println((int)this ->state);

    case P_IDLE: {
      // Elevator is stopped, doors are closed. Time to find a new job.
      
      // First, check if there's a request AT the current floor
      if (stops[currentLogicalFloor] || upStops[currentLogicalFloor] || downStops[currentLogicalFloor]) {
        Serial.println("Phys: Request at current floor. Opening doors.");
        physicalState = P_OPENING_DOOR;
        LCD::physicalState = P_OPENING_DOOR;
        LCD::updateLcd();
        // NOTE: We no longer set doorTimer here
      } 
      else {
        // If no job here, ask the logic class what to do
        this -> changeElevatorMoving(); // Updates elev.state
        
        // RETTET: Bruker "::State::" istedenfor ".State::"
        if (state != Elevator::State::Idle) {
          // Logic class wants to move. Find the *next* stop.
          physicalTargetFloor = findNextTargetInDirection();
          
          if (physicalTargetFloor != -1) {
            // We have a target!
            Serial.print("Phys: New target acquired: ");
            Serial.println(physicalTargetFloor);
            physicalState = P_MOVING;
            LCD::physicalState = P_MOVING;
            LCD::updateLcd();
            // Reset PID
            PID::integral = 0;
            PID::lastError = 0;
            PID::lastTime = millis();

            if (physicalTargetFloor > currentLogicalFloor)
              this ->state = Elevator::State::MovingUp;
            else if (physicalTargetFloor < currentLogicalFloor)
              this ->state = Elevator::State::MovingDown;
             else
              this ->state = Elevator::State::Idle;
          } else {
            // Logic state is moving, but no target found?
            // This can happen if only a request at current floor exists
            // which was missed. Force logic back to Idle.
            // RETTET: Bruker "::State::" istedenfor ".State::"
            this ->state = Elevator::State::Idle;
          }
        }
      }
      LCD::currentLogicalFloor = currentLogicalFloor;
      LCD::physicalTargetFloor = physicalTargetFloor;
      LCD::physicalState = physicalState;


      break;
    }

   case P_MOVING: {
    long targetPulses = physicalTargetFloor * DCmotor::PulsesPerRevolution;
    PID::computePID(targetPulses, DCmotor::EncoderCount);
    analogWrite(DCmotor::PWMpin, DCmotor::PwmValue);

    digitalWrite(DCmotor::directionPin, DCmotor::MotorDirection ? HIGH : LOW);

    currentLogicalFloor = round((float)DCmotor::EncoderCount / DCmotor::PulsesPerRevolution); 
    LCD::currentLogicalFloor = currentLogicalFloor;
    LCD::physicalTargetFloor = physicalTargetFloor;
    LCD::physicalState = P_MOVING;

    // Sjekk om vi er innenfor toleranse
    if (abs((float)(targetPulses - DCmotor::EncoderCount) / DCmotor::PulsesPerRevolution) < DCmotor::floorTolerance) {
        DCmotor::PwmValue = 0;
        analogWrite(DCmotor::PWMpin, 0);

        currentLogicalFloor = physicalTargetFloor;
        currentFloor = currentLogicalFloor;

        Serial.print("Phys: Arrived at target floor ");
        Serial.println(currentLogicalFloor);

        physicalState = P_OPENING_DOOR;
        LCD::physicalState = P_OPENING_DOOR;
      
    }
    break;
}
    case P_OPENING_DOOR:
    
      delay(200); // Small pause before door motor

      LCD::physicalState = P_OPENING_DOOR;
      LCD::updateLcd(); 
      LCD::lastLcdUpdate = millis(); // Reset LCD timer

      Stepper::OpenDoor(); // This is the blocking call

      clearRequestsAt(::currentLogicalFloor);

      physicalState = P_DOOR_OPEN;
      LCD::physicalState = P_DOOR_OPEN;
      doorTimer = millis();
      break;

    
    case P_DOOR_OPEN: 
      if (millis() - doorTimer > doorOpenTime) {
      
        physicalState = P_CLOSING_DOOR;
        LCD::physicalState = P_CLOSING_DOOR;
      }
      break;

    
    case P_CLOSING_DOOR:
      LCD::updateLcd();
      LCD::lastLcdUpdate = millis(); 

      Stepper::CloseDoor();

      LCD::physicalState = P_IDLE;
      physicalState = P_IDLE;
      Serial.println("Phys: Ready for next job.");
      break;
  }
}



 



