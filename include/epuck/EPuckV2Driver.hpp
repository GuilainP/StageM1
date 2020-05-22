#pragma once

#define EPUCK_PORT 1000

#define DEBUG_CONNECTION_INIT 0
#define DEBUG_UPDATE_SENSORS_DATA 1
#define DEBUG_ODOMETRY 0
#define DEBUG_IMU 0
#define DEBUG_SPEED_RECEIVED 0
#define DEBUG_LED_RECEIVED 0
#define DEBUG_RGB_RECEIVED 0
#define DEBUG_MAG_FIELD 0
#define DEBUG_BATTERY 0

#define READ_TIMEOUT_SEC 10
#define READ_TIMEOUT_USEC 0
#define MAX_CONNECTION_TRIALS 3

#define WHEEL_DIAMETER 4        // cm.
#define WHEEL_SEPARATION 5.3    // Separation between wheels (cm).
#define WHEEL_DISTANCE 0.053    // Distance between wheels in meters (axis length); it's the same value as "WHEEL_SEPARATION" but expressed in meters.
#define WHEEL_CIRCUMFERENCE ((WHEEL_DIAMETER*M_PI)/100.0)    // Wheel circumference (meters).
#define MOT_STEP_DIST (WHEEL_CIRCUMFERENCE/1000.0)      // Distance for each motor step (meters); a complete turn is 1000 steps (0.000125 meters per step (m/steps)).
#define ROBOT_RADIUS 0.035 // meters.

#define LED_NUMBER 6 // total number of LEDs on the robot (0,2,4,6=leds, 8=body, 9=front) 
#define RGB_LED_NUMBER 4

#define DEG2RAD(deg) (deg / 180 * M_PI)
#define GYRO_RAW2DPS (250.0/32768.0f)   //250DPS (degrees per second) scale for int16 raw value
#define STANDARD_GRAVITY 9.80665f

#define RESISTOR_R1             220 //kohm
#define RESISTOR_R2             330 //kohm
#define VOLTAGE_DIVIDER         (1.0f * RESISTOR_R2 / (RESISTOR_R1 + RESISTOR_R2))
#define VREF                    3.0f //volt correspond to the voltage on the VREF+ pin
#define ADC_RESOLUTION          4096
#define COEFF_ADC_TO_VOLT       ((1.0f * ADC_RESOLUTION * VOLTAGE_DIVIDER) / VREF) //convertion from adc value to voltage
#define MAX_VOLTAGE				4.2f	//volt
#define MIN_VOLTAGE				3.4f	//volt

const double prox_range_min = 0.005+ROBOT_RADIUS; // 0.5 cm + ROBOT_RADIUS.
const double prox_range_max = 0.05+ROBOT_RADIUS; // 5 cm + ROBOT_RADIUS. 
const double prox_angle_min = -M_PI/2.0;
const double prox_angle_max = M_PI/2.0;
const double prox_angle_increment = M_PI/18.0; // 10 degrees.




