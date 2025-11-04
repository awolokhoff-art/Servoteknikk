#ifndef ELEVATOR_H
#define ELEVATOR_H

#include <Arduino.h>

enum class Request { Down, Up };

class Elevator {
public:
    enum class State { MovingDown = 0, MovingUp = 1, Idle = 2 };

    static const int MAX_FLOOR = 7;
    static const int NUM_FLOORS = MAX_FLOOR + 1;

    int currentFloor = 0;
    int lowerFloor = 0;
    int upperFloor = MAX_FLOOR;
    State state = State::Idle;

    Elevator(State state_ = State::Idle, int current = 0);

    bool addCarRequest(int floor);
    bool addHallRequest(int floor, Request request);

    void changeElevatorMoving();
    bool stopAtNextFloor();
    int nextFloor();
    void clearStops();

    bool hasAnyStops() const;
    bool acceptedFloorRange(int floor) const;
    void setStop(int floor);

    // --- **Gjør arrayene public her** ---
    bool upStops[NUM_FLOORS];
    bool downStops[NUM_FLOORS];
    bool stops[NUM_FLOORS];

    void startElevatorMoving(int floor);
    bool hasStopsUpFromAny(int floor) const;
    bool hasStopsDownFromAny(int floor) const;
};

#endif
 

