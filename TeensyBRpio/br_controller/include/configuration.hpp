#ifndef _CONFIGURATION_HPP_
#define _CONFIGURATION_HPP_

/* PID */
// See classes Integral, Derivative and ProportionalIntegralDerivative (folder math)

#define DEFAULT_KP 13.2
#define DEFAULT_TI 0.25
#define DEFAULT_TD 0.167
#define DERIVATIVE_FILTER 5.0

// Use {} instead of a number to disable saturation
#define INTEGRAL_SATURATION 0.5
#define DERIVATIVE_SATURATION 10.0
#define PID_SATURATION 10.0

/* CONTROLLER */
// See controller/UnicycleController.hpp and manager/ControllerManager.hpp

// x offset of the tracking point (should be strictly greater than 0)
#define ASSERV_ALPHA 0.10 // m
// y offset of the the tracking point (can be set to 0 if the point is centered)
#define ASSERV_BETA 0.0 // m

#define UPDATE_INTERVAL 5000 // µS (NOT milliseconds)

/* TRAJECTORIES */
// TODO check the speeds and accelerations on the real robot

#define MAX_LINEAR_GOAL_SPEED 0.5   // m/s
#define MAX_ROTATION_GOAL_SPEED 3.0 // rad/s

#define DEFAULT_LINEAR_ACCELERATION 0.1   // m/s^2
#define DEFAULT_ROTATION_ACCELERATION 3.0 // rad/s^2

#define BRAKING_LINEAR_ACCELERATION 1.5   // m/s^2, >= DEFAULT_LINEAR_ACCELERATION
#define BRAKING_ROTATION_ACCELERATION 3.0 // rad/s^2, >= DEFAULT_ROTATION_ACCELERATION

// threshold to determine if the robot is moving or stopped
#define STOPPED_SPEED_THRESHOLD 0.001 // m/s // TODO set

/* ROS */

#ifdef ARDUINO
// Also applies to topic /odos_count
#define SEND_POSITION_INTERVAL 100 // ms
#else
#define SEND_POSITION_INTERVAL 10 // ms (for a smooth GUI)
#endif

// Applies to topic /logTotaleArray
#define ROS_LOG_INTERVAL 100 // ms

/* ODOS (See feedback/PositionEstimatorOdo.hpp) - Not used by simulation */

#define ODOS_METHOD MethodMoveFirst

// Calibrated values
#define ECARTS_ODOS 5978.462898951549L     //5980.73537126610L // (ticks.rad^(-1) ecart entre les 2 odos
#define UNITS_ODOS 51.56183449886886L //51.54179961710274L // ticks.mm^(-1)
#define L_R_ODOS 0.9983401020305469L //1.0011809854125424L  // Correction factor between the encoders

/* WHEELS */

#define WHEEL_DISTANCE 0.22  // m
#define WHEEL_DIAMETER 0.06  // m
#define TRANSMISSION_RATIO 1 // reduction factor (not used by simulation)
#define MAX_MOTOR_SPEED 8    // turns/s (not used by simulation)

/* PINS - Not used by simulation */

#define ODRIVE_RX_PIN 0
#define ODRIVE_TX_PIN 1
// The pin number and blinking interval of the LED (Arduino only) are defined in lib/Led/Led.cpp

/* SIMULATION - Not used on the Teensy */

#ifndef ARDUINO
#define NOISE_STD_DEV 0.1
#else
#define NOISE_STD_DEV 0 // random number generator not working on the Teensy
#endif
#define SIM_MIN_WHEEL_SPEED 0.2 // rad/s (simulates static friction) // TODO measure actual value
#define SIM_MAX_WHEEL_SPEED 50  // rad/s (simulates saturation, roughly corresponds to MAX_MOTOR_SPEED)
#define SIM_MAX_WHEEL_ACCEL 100 // rad/s² (simulates damping + robot inertia) // TODO measure actual value

/* MEMORY POOL config - Arduino only */
// The memory pool contains all the "SmallDeque":
//    - The pre-generated Bézier curves (See CurveTrajectory and MultiCurveTrajectory)
//    - For each curve, the number of sample points and the number of curvature extrema (see CurveSampling and RollingMax)
#define BLOCK_COUNT 64
#define BLOCK_SIZE 64 // bytes

#endif