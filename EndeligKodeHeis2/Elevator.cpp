#include "Elevator.h"
#include <Arduino.h>


Elevator::Elevator(State state_, int current)
    : currentFloor(current), state(state_), lowerFloor(0), upperFloor(MAX_FLOOR)
{
    clearStops();
}

bool Elevator::addCarRequest(int floor) {
    if (!acceptedFloorRange(floor)) return false;
    if (floor == currentFloor && state == State::Idle) return false;

    stops[floor] = true;
    startElevatorMoving(floor);
    return true;
}

// I Elevator.cpp

bool Elevator::addHallRequest(int floor, Request request) {
    if (!acceptedFloorRange(floor)) return false;
    // Ignorer kallet hvis vi er i etasjen og i ro
    if (floor == currentFloor && state == State::Idle) return false; 

    // --- **NY VALIDERING HER** ---
    // Kan ikke kalle OPP fra øverste etasje (f.eks. 7u)
    if (floor == upperFloor && request == Request::Up) return false;
    // Kan ikke kalle NED fra nederste etasje (f.eks. 0d)
    if (floor == lowerFloor && request == Request::Down) return false;
    // ----------------------------

    // Korrigert logikk fra forrige gang
    if (request == Request::Up) {
        upStops[floor] = true;
    }
    if (request == Request::Down) {
        downStops[floor] = true;
    }

    startElevatorMoving(floor); // Fortell heisen at den må begynne å bevege seg
    return true;
}

void Elevator::changeElevatorMoving() {
    if (!hasAnyStops()) {
        state = State::Idle;
        return;
    }

    if (state == State::MovingUp) {
        if (hasStopsUpFromAny(currentFloor)) return;
        state = State::MovingDown;
        return;
    }

    if (state == State::MovingDown) {
        if (hasStopsDownFromAny(currentFloor)) return;
        state = State::MovingUp;
        return;
    }

    if (state == State::Idle) {
        if (hasStopsUpFromAny(currentFloor))
            state = State::MovingUp;
        else if (hasStopsDownFromAny(currentFloor))
            state = State::MovingDown;
    }
}

void Elevator::clearStops() {
    for (int i = 0; i < NUM_FLOORS; ++i) {
        stops[i] = upStops[i] = downStops[i] = false;
    }
}

bool Elevator::hasAnyStops() const {
    for (int i = 0; i < NUM_FLOORS; ++i)
        if (stops[i] || upStops[i] || downStops[i]) return true;
    return false;
}

void Elevator::setStop(int floor) {
    if (acceptedFloorRange(floor)) stops[floor] = true;
}

bool Elevator::acceptedFloorRange(int floor) const {
    return floor >= lowerFloor && floor <= upperFloor;
}

void Elevator::startElevatorMoving(int floor) {
    if (state == State::Idle) {
        if (currentFloor < floor) state = State::MovingUp;
        else if (currentFloor > floor) state = State::MovingDown;
    }
}


bool Elevator::hasStopsUpFromAny(int floor) const {
    for (int i = floor + 1; i <= upperFloor; ++i) if (upStops[i] || downStops[i] || stops[i]) return true;
    return false;
}

bool Elevator::hasStopsDownFromAny(int floor) const {
    for (int i = floor - 1; i >= lowerFloor; --i) if (upStops[i] || downStops[i] || stops[i]) return true;
    return false;
}


