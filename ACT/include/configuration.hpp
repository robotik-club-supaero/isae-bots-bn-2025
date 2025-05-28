#ifndef _CONFIGURATION_HPP_
#define _CONFIGURATION_HPP_

// -- SERVO POSITIONS --

// Clamp 1
#define CLAMP_2_1_OPEN_POS 10   // TODO
#define CLAMP_2_1_CLOSED_POS 160  // TODO

// Clamp 2
#define CLAMP_2_2_OPEN_POS 10   // TODO
#define CLAMP_2_2_CLOSED_POS 160  // TODO

// Clamp bas  
#define CLAMP_1_1_OPEN_POS 5   // TODO
#define CLAMP_1_1_CLOSED_POS 170  // TODO

// Clamp en trop  
#define CLAMP_1_2_OPEN_POS 0   // TODO
#define CLAMP_1_2_CLOSED_POS 0  // TODO

// The servo that opens first when deploying the banner
#define BANNER_1_DEPLOYED_POS 50  // TODO
#define BANNER_1_RETRACTED_POS 5  // TODO

// The other servo of the banner
#define BANNER_2_DEPLOYED_POS 50  // TODO
#define BANNER_2_RETRACTED_POS 5  // TODO

// -- STEPPERS CONFIGURATION (ELEVATORS) --

#define ELEVATOR_1_STEP_PER_REV 200 // TODO
#define ELEVATOR_2_STEP_PER_REV ELEVATOR_1_STEP_PER_REV // TODO

#define ELEVATOR_1_SPEED 800 // rev per minute // TODO
#define ELEVATOR_2_SPEED ELEVATOR_1_SPEED // rev per minute // TODO

// number of steps between states DOWN and MIDDLE - change sign to invert direction
#define ELEVATOR_1_POS_MIDDLE 4000
#define ELEVATOR_2_POS_MIDDLE 10000 // TODO HAUT 43000

// number of steps between states DOWN and UP - must be same sign as POS_MIDDLE and greater (absolutely)
// WARNING: this is between DOWN and UP, NOT between MIDDLE and UP
#define ELEVATOR_1_POS_UP 5000 // TODO butée     BAS
#define ELEVATOR_2_POS_UP 30000 // TODO HAUT 43000

// -- PINS --

#define ELEVATOR_1_STEP_PIN 3
#define ELEVATOR_1_DIR_PIN 4

#define ELEVATOR_2_STEP_PIN 5
#define ELEVATOR_2_DIR_PIN 6

#define CLAMP_1_1_PIN 20 
#define CLAMP_1_2_PIN 21
#define CLAMP_2_1_PIN 22
#define CLAMP_2_2_PIN 23 

#define BANNER_1_PIN 9
#define BANNER_2_PIN 10

#define BUMPER_1_PIN 0 // TODO
#define BUMPER_2_PIN 0 // TODO

// -- BUMPER CONFIG --

#define BUMPER_DETECTION_THRESHOLD 0.7 // 0 (very sensitive) -> 1 (never detects)
#define BUMPER_FILTER_TAU 0.05 // s (time constant of the low-pass filter)
#define BUMPER_UPDATE_INTERVAL 10 // ms

// -- OTHER CONFIG --

#define CALLBACK_INTERVAL 10  // ms
#define BANNER_INTERVAL 1000    // ms // TODO

// If moving the elevator takes more time than the timeout, `ElevatorStepper::loop` returns anyway after (roughly) the timeout, so ROS orders can
// still be processed and other actuators' state can be updated.
// The displacement continues the next time `ElevatorStepper::loop` is called.
#define STEPPER_YIELD_TIMEOUT 50 // ms


#endif