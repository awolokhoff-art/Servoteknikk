#ifndef ELEVATOR_H
#define ELEVATOR_H

#include <Arduino.h>

extern int currentLogicalFloor;

class Elevator {

private: 
    static const int MAX_FLOOR = 7;
    static const int NUM_FLOORS = MAX_FLOOR + 1;
    
    int findNextTargetInDirection();
    int getCurrentLogicalFloor() const { return ::currentLogicalFloor; }

    bool stopAtNextFloor();
    bool hasStopsUpFromAny(int floor) const;
    bool hasStopsDownFromAny(int floor) const;
    bool acceptedFloorRange(int floor) const;

    void clearStops();
    void setStop(int floor);
    void clearRequestsAt(int floor);
    void changeElevatorMoving();
    void startElevatorMoving(int floor);
 

public:
    enum class State { MovingDown = 0, MovingUp = 1, Idle = 2 };
    enum class Request { Down, Up };
    
    Elevator(State state_ = State::Idle, int current = 0);

    int currentFloor = 0;
    int lowerFloor = 0;
    int upperFloor = MAX_FLOOR;
    
    State state = State::Idle;
    
    // Request added in three queues
    bool upStops[NUM_FLOORS];
    bool downStops[NUM_FLOORS];
    bool stops[NUM_FLOORS];
    
    bool hasAnyStops() const;
    bool addCarRequest(int floor);
    bool addHallRequest(int floor, Request request);

    void isRequest();
    void processPhysicalState();
     

};
#endif
 

