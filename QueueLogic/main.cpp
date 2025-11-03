#include <iostream>
#include <set>
#include <algorithm>
#include <cmath>
#include <limits>
#include <math.h>

enum class Request {Down, Up};

struct Elevator {

    enum class State {MovingDown = 0, MovingUp = 1, Idle = 2};
    int currentFloor = 0;
    int lowerFloor = 0;
    int upperFloor = 8;

    State state = State::Idle;

    Elevator(State state_, int current, const std::set<int>& up,const std::set<int>& down, const std::set<int>& stops_):
    state(state_), currentFloor(current), upStops(up), downStops(down), stops(stops_) {}

    std::set<int> upStops;
    std::set<int> downStops;
    std::set<int> stops;

    // Forteller om heisen går opp eller ned
    void startElevatorMoving(int floor) {
        if (state == State::Idle) {
            if (currentFloor < floor) {
                state = State::MovingUp;
                std::cout << "Moving up to " << floor << " from " << currentFloor  << std::endl;
            }
            else {
                state = State::MovingDown;
                std::cout << "Moving down to " << floor << " from " << currentFloor  << std::endl;
            }
        }
    }
    //Sjekker om det er flere stops oppover
    bool hasStopsUp(int floor, const std::set<int>& stops) {
        return stops.upper_bound(floor) != stops.end();
    }
    //Sjekker om det er flere stops nedover
    bool hasStopsDown(int floor, const std::set<int>& stops) {
        for (int i = floor; i >= 0; --i) {
            if (stops.find(i) != stops.end()) {
                return true;
            }
        }
        return false;
    }
    bool acceptedFloorRange(int floor) {
        if (floor >= lowerFloor && floor <= upperFloor) {
            return true;
        }
        std::cout << "You have entered a floor outside of the scoope, try again" << std::endl;
        return false;

    }
    // Ender retningen basert om heisen er "tom", går opp for å se gå ned vv.
    void changeElevatorMoving() {
        if (stops.empty() && upStops.empty() && downStops.empty()) {
            state = State::Idle;
            std::cout << "Stoping elevator"<< std::endl;
        }
        if (state == State::MovingUp) {
            if (hasStopsUp(currentFloor, upStops)) {
                return;
            }
            if (hasStopsUp(currentFloor, stops)) {
                return;
            }
            state = State::MovingDown;
            std::cout << "Start moving down " << std::endl;
            return;
        }
        if (state == State::MovingDown) {
            if (hasStopsDown(currentFloor, downStops)) {
                return;
            }
            if (hasStopsDown(currentFloor, stops)) {
                return;
            }
            state = State::MovingUp;
            std::cout << "Start moving up " << std::endl;
            return;
        }
    }
    // bare for innsiden av heisen, legger til etasjer i stops
    void addCarRequest(int floor) {
        if (acceptedFloorRange(floor) != true) {
            return;
        }
        if (floor == currentFloor){
            return;
        }
        stops.insert(floor);
        startElevatorMoving(floor);
    }
    //Bare for utsiden av heisen, hvor direction er viktig, legges inn i upStops og downStops.
    void addHallRequest(int floor, Request request) {
        std::cout << "in addHallRequest" << std::endl;
        if (acceptedFloorRange(floor) != true)
            return;

        if (floor == currentFloor) {
            return; //open doors
        }
        if (request == Request::Up) {
            upStops.insert(floor);
        }
        if (currentFloor < floor) {
            upStops.insert(floor);
        }

        if (request == Request::Down) {
            downStops.insert(floor);
        }
        if (currentFloor > floor) {
            downStops.insert(floor);
        }

        startElevatorMoving(floor);
    }
    // må kjøres kontiunerlig, sjekker for hvert etasje om den skal stoppe eller ikke
    bool stopAtNextFloor() {
        if (state == State::MovingUp) {
            std::cout << "Moving up " << currentFloor  << std::endl;
            ++currentFloor;

            if (upStops.find(currentFloor) != upStops.end()) {
                upStops.erase(currentFloor);
                return true;
            }
        }
        if (state == State::MovingDown ) {
            std::cout << "Moving down " << currentFloor  << std::endl;
            --currentFloor;
            if (downStops.find(currentFloor) != downStops.end()) {
                downStops.erase(currentFloor);
                return true;
            }
        }
        if (stops.find(currentFloor) != stops.end()) {
            stops.erase(currentFloor);
            return true;
        }
        return false;
    }
    // Gir ut hva neste etasje heisen skal stoppe på
    int nextFloor() {
        while (!stopAtNextFloor()) {}
        std::cout << "Next Floor is " << currentFloor << std::endl;
        return currentFloor;

    }
};
// enkel test for å sjekke heisen opererer som den skal
bool test(Elevator::State state_, int current,
    const std::set<int>& up,
    const std::set<int>& down,
    const std::set<int>& stops_,
    int hallrequest,
    Request request,
    int carRequest,
    int nextfloor) {

    Elevator elevator(state_, current, up, down, stops_);

    if (hallrequest >= 0) {
        elevator.addHallRequest(hallrequest,request);
    }
    if (carRequest >= 0) {
        elevator.addCarRequest(carRequest);
    }

    if (elevator.nextFloor() != nextfloor) {
        std::cout<<"Failed"<<std::endl;
        return false;
    }

    std::cout<<"Success!"<<std::endl;
    return true;
}

int main() {
    std::cout<<"\nTest 1"<<std::endl;
    test(Elevator::State::Idle, 0,{},{},{},3,Request::Up,-1,3);
    std::cout<<"\nTest 2"<<std::endl;
    test(Elevator::State::Idle, 0,{},{},{},3,Request::Up,-1,6);
    std::cout<<"\nTest 3"<<std::endl;
    test(Elevator::State::MovingUp, 4,{3,6,7},{6,4},{7,8,1},1,Request::Up,-1,6);
    std::cout<<"\nTest 4"<<std::endl;
    test(Elevator::State::Idle, 5,{},{},{},1,Request::Down,{},1);
    std::cout<<"\nTest 5"<<std::endl;
    test(Elevator::State::Idle, 1,{},{},{},5,Request::Up,{},5);
    std::cout<<"\nTest 6"<<std::endl;
    test(Elevator::State::Idle, 5,{},{},{},1,Request::Up,{},1);
    std::cout<<"\nTest 7"<<std::endl;
    test(Elevator::State::Idle, 1,{},{},{},5,Request::Down,{},5);

    std::cout<<"\nTest 8"<<std::endl;
    test(Elevator::State::Idle, 3,{},{},{},2,Request::Up,{},2);

    std::cout<<"\nTest 9"<<std::endl;
    test(Elevator::State::MovingUp, 6,{},{},{7,8},3,Request::Up,{},7);

    std::cout<<"\nTest 10"<<std::endl;
    test(Elevator::State::MovingUp, 4,{},{},{8},6,Request::Up,{},6);
    return 0;

}

