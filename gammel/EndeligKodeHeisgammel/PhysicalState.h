#ifndef PHYSICALSTATE_H
#define PHYSICALSTATE_H

#include "Elevator.h"

// Enum for fysisk tilstand til heisen
enum PhysicalState {
    P_IDLE,
    P_MOVING,
    P_OPENING_DOOR,
    P_DOOR_OPEN,
    P_CLOSING_DOOR
};

#endif
