// Elevator.h
#ifndef ELEVATOR_H
#define ELEVATOR_H

#include <Arduino.h>
//in .ino #include "Elevator.h", and call Serial.begin(115200) in setup;
//Use the class like Elevator elev; elev.addHallRequest(3, Request::Up); int f = elev.nextFloor();

enum class Request { Down, Up };

class Elevator {
public:
  enum class State { MovingDown = 0, MovingUp = 1, Idle = 2 };

  // Configure number of floors here (0..MAX_FLOOR)
  static const int MAX_FLOOR = 8; // keeps same as your original upperFloor = 8
  static const int NUM_FLOORS = MAX_FLOOR + 1;

  // public members
  int currentFloor = 0;
  int lowerFloor = 0;
  int upperFloor = MAX_FLOOR;
  State state = State::Idle;

  // constructor (default)
  Elevator(State state_ = State::Idle, int current = 0)
    : currentFloor(current), state(state_) {
    clearStops();
  }

  // add a stop from inside the car
  void addCarRequest(int floor) {
    if (!acceptedFloorRange(floor)) return;
    if (floor == currentFloor) return;
    stops[floor] = true;
    startElevatorMoving(floor);
  }

  // add a hall request (outside), with direction
  void addHallRequest(int floor, Request request) {
    Serial.println(F("in addHallRequest"));
    if (!acceptedFloorRange(floor)) return;
    if (floor == currentFloor) {
      // would open doors in real system
      return;
    }

    if (request == Request::Up) {
      upStops[floor] = true;
    }
    // keep consistent with original: requests may be inserted also depending on currentFloor relation
    if (currentFloor < floor) {
      upStops[floor] = true;
    }

    if (request == Request::Down) {
      downStops[floor] = true;
    }
    if (currentFloor > floor) {
      downStops[floor] = true;
    }

    startElevatorMoving(floor);
  }

  // call repeatedly - returns true if elevator stops at the new currentFloor
  bool stopAtNextFloor() {
    if (state == State::MovingUp) {
      Serial.print(F("Moving up "));
      Serial.println(currentFloor);
      // move one floor up
      if (currentFloor < upperFloor) ++currentFloor;

      if (upStops[currentFloor]) {
        upStops[currentFloor] = false;
        return true;
      }
    }

    if (state == State::MovingDown) {
      Serial.print(F("Moving down "));
      Serial.println(currentFloor);
      if (currentFloor > lowerFloor) --currentFloor;

      if (downStops[currentFloor]) {
        downStops[currentFloor] = false;
        return true;
      }
    }

    if (stops[currentFloor]) {
      stops[currentFloor] = false;
      return true;
    }

    return false;
  }

  // blocks until next stop and returns the floor number (keeps original behaviour)
  // Note: this is blocking like your original nextFloor(); in an Arduino sketch you
  // might prefer a non-blocking loop-based approach.
  int nextFloor() {
    // Safety: prevent an infinite tight-loop causing watchdog resets if there are no stops.
    // This preserves original semantic (while (!stopAtNextFloor()) {}) but is still blocking.
    unsigned long loopStart = millis();
    const unsigned long TIMEOUT_MS = 20000UL; // 20 seconds fallback if something's wrong
    while (!stopAtNextFloor()) {
      // attempt to change direction/state if no stops in current direction
      changeElevatorMoving();

      // small delay to avoid burning CPU in Arduino
      delay(10);

      // If there are no stops at all, break to avoid infinite loop (keeps device responsive).
      if (!hasAnyStops()) {
        Serial.println(F("No stops left - breaking nextFloor loop"));
        break;
      }

      // fallback timeout
      if (millis() - loopStart > TIMEOUT_MS) {
        Serial.println(F("nextFloor timeout - breaking loop"));
        break;
      }
    }

    Serial.print(F("Next Floor is "));
    Serial.println(currentFloor);
    return currentFloor;
  }

  // change direction/state when needed (based on remaining stops)
  void changeElevatorMoving() {
    if (!hasAnyStops()) {
      state = State::Idle;
      Serial.println(F("Stopping elevator"));
      return;
    }

    if (state == State::MovingUp) {
      if (hasStopsUp(currentFloor)) return;
      if (hasStopsUpFromAny(currentFloor)) return;
      state = State::MovingDown;
      Serial.println(F("Start moving down"));
      return;
    }

    if (state == State::MovingDown) {
      if (hasStopsDown(currentFloor)) return;
      if (hasStopsDownFromAny(currentFloor)) return;
      state = State::MovingUp;
      Serial.println(F("Start moving up"));
      return;
    }

    // If Idle but there are stops, decide direction based on nearest/requests:
    if (state == State::Idle) {
      // simple policy: if there is any stop above => move up, else move down
      if (hasStopsUpFromAny(currentFloor) || hasStopsUp(currentFloor)) {
        state = State::MovingUp;
        Serial.println(F("Idle -> start moving up"));
      } else if (hasStopsDownFromAny(currentFloor) || hasStopsDown(currentFloor)) {
        state = State::MovingDown;
        Serial.println(F("Idle -> start moving down"));
      }
    }
  }

  // call to reset all stops
  void clearStops() {
    for (int i = 0; i < NUM_FLOORS; ++i) {
      upStops[i] = false;
      downStops[i] = false;
      stops[i] = false;
    }
  }

  // simple helpers
  bool hasAnyStops() const {
    for (int i = 0; i < NUM_FLOORS; ++i) {
      if (upStops[i] || downStops[i] || stops[i]) return true;
    }
    return false;
  }

  // For testing convenience: set a raw stop
  void setStop(int floor) {
    if (acceptedFloorRange(floor)) stops[floor] = true;
  }

  // internal stop storage (arrays indexed by floor number)
  bool upStops[NUM_FLOORS];
  bool downStops[NUM_FLOORS];
  bool stops[NUM_FLOORS];

  // start moving based on a requested floor
  void startElevatorMoving(int floor) {
    if (state == State::Idle) {
      if (currentFloor < floor) {
        state = State::MovingUp;
        Serial.print(F("Moving up to "));
        Serial.print(floor);
        Serial.print(F(" from "));
        Serial.println(currentFloor);
      } else if (currentFloor > floor) {
        state = State::MovingDown;
        Serial.print(F("Moving down to "));
        Serial.print(floor);
        Serial.print(F(" from "));
        Serial.println(currentFloor);
      } else {
        // same floor -> remain idle (doors would open)
      }
    }
  }

  // check if there are stops strictly above given floor in upStops array
  bool hasStopsUp(int floor) const {
    for (int i = floor + 1; i <= upperFloor; ++i) {
      if (upStops[i]) return true;
    }
    return false;
  }

  // check if any stops above in any stops array (used in changeElevatorMoving)
  bool hasStopsUpFromAny(int floor) const {
    for (int i = floor + 1; i <= upperFloor; ++i) {
      if (upStops[i] || stops[i] || downStops[i]) return true;
    }
    return false;
  }

  // check if there are stops at or below floor in downStops array
  bool hasStopsDown(int floor) const {
    for (int i = floor - 1; i >= lowerFloor; --i) {
      if (downStops[i]) return true;
    }
    return false;
  }

  // check if any stops below in any stops array
  bool hasStopsDownFromAny(int floor) const {
    for (int i = floor - 1; i >= lowerFloor; --i) {
      if (downStops[i] || stops[i] || upStops[i]) return true;
    }
    return false;
  }

  // check valid floor range
  bool acceptedFloorRange(int floor) const {
    if (floor >= lowerFloor && floor <= upperFloor) return true;
    Serial.println(F("You have entered a floor outside of the scope, try again"));
    return false;
  }
};

#endif // ELEVATOR_H
