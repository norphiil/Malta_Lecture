#include <Arduino.h>
#include "FastLED.h"
#include "device_gyro_accel.h"

enum IRSensor
{
    IR_LEFT,
    IR_MIDDLE,
    IR_RIGHT
};

class IR
{
public:
    // Sets the pins and default threshold (BLACK_ON_WHITE_THRESHOLD)
    void init(void);
    // Sets a custom threshold
    void setThreshold(uint16_t thresholdL, uint16_t thresholdM, uint16_t thresholdR);
    void setThreshold(uint16_t threshold[3]);
    // Logs the values of the 3 sensors
    void test(void);

    // Returns true if the provided sensor senses a line
    bool get(IRSensor sensor);

    // Returns the actual reading of the sensor
    uint16_t value(IRSensor sensor);
    static bool inInterval(long x, long s, long e);
    // THRESHOLDS
    const uint16_t BLACK_ON_WHITE_THRESHOLDS[3] = {800, 750, 800};
    // ...

private:
    // value at which the robot will recognise a line
    uint16_t thresholdL, thresholdM, thresholdR;
    const uint8_t PIN_IR_L = A2; // Left IR Sensor
    const uint8_t PIN_IR_M = A1; // Middle IR Sensor
    const uint8_t PIN_IR_R = A0; // Right IR Sensor
};

class Servo
{
public:
    void init(void);

    void setAngle(uint8_t new_angle);
    void setAngleBrute(uint8_t new_angle);
    // uint8_t find_way(Ultrasonic ultrasonic);

private:
    uint8_t angle;

#define PIN_SERVO 10 // Pin responsible for the servo motor
};

class Ultrasonic
{
public:
    void init(void);
    // test to get the distance every second
    void test(void);
    uint16_t get(void);

private:
    unsigned int microseconds_to_cm(unsigned int microseconds);
#define PIN_TRIG 13      // Pin responsible for the trigger pulse. LOW = prepare for pulse, HIGH = send pulse
#define PIN_ECHO 12      // Pin which gives us the time until the echo pulse was received, HIGH = receive the value
#define MAX_DISTANCE 400 // cm
};

enum Direction
{
    FORWARDS,
    BACKWARDS,
    RIGHT,
    LEFT,
    FORWARDS_RIGHT,
    FORWARDS_LEFT,
    BACKWARDS_RIGHT,
    BACKWARDS_LEFT,
    STOP
};

class Motor // TB6612
{
public:
    // Initialises the pins
    void init(void);

    void test(void);

    // One function for all simple movements. Returns false if the direction is not recognised, true otherwise
    void move(Direction direction, uint8_t speed);
    void stop(void);

    // These are the functions responsible for changing pin values, can use them specifically to move non-predefined ways
    void rightMotor(uint8_t direction, uint8_t speed);
    void leftMotor(uint8_t direction, uint8_t speed);
    void linearMotionControl(uint8_t direction, uint8_t directionRecord, uint8_t speed, uint8_t Kp, uint8_t UpperLimit, uint8_t LowerLimit);
    void motorControl(uint8_t direction_A, uint8_t speed_A, uint8_t direction_B, uint8_t speed_B, uint8_t controlED);
    void startTracking(void);
    void flashLed(void);
    IR getIr(void);
    GyroAccel getGyroAccel(void);
    Ultrasonic getUltrasonic(void);
    Servo getServo(void);

private:
    // Clamps the input speed between the MIN_SPEED and MAX_SPEED
    uint8_t normaliseSpeed(uint8_t speed);

    // These functions abstract from the leftMotor and rightMotor functions to provide direction
    void forwards(uint8_t speed);
    void backwards(uint8_t speed);
    void right(uint8_t speed);
    void left(uint8_t speed);
    void forwardsRight(uint8_t speed);
    void forwardsLeft(uint8_t speed);
    void backwardsRight(uint8_t speed);
    void backwardsLeft(uint8_t speed);

    IR ir;
    GyroAccel gyroAccel;
    Ultrasonic ultrasonic;
    Servo servo;
    uint8_t isIntersectionCounter = 0;
    uint8_t forwardCounter = 0;
    Direction lastDirection = STOP;
    unsigned long lastTime = 0;

#define PIN_MOTOR_A_PWM 5
#define PIN_MOTOR_A_IN 7
#define PIN_MOTOR_B_PWM 6
#define PIN_MOTOR_B_IN 8
#define PIN_MOTOR_STBY 3

#define MAX_SPEED 255
#define MIN_SPEED 30 // Approximately the minimum speed the robot will still move at from our testing

#define MOTOR_FORWARDS 1  // 1 and 0 are the same as HIGH and LOW, these are used in the leftMotor and rightMotor functions
#define MOTOR_BACKWARDS 0 // to indicate the direction
};

class Cell
{
public:
    Cell() : leftWall(false), rightWall(false), topWall(false), bottomWall(false), value(0) {}
    void setLeftWall(bool wall);
    void setRightWall(bool wall);
    void setTopWall(bool wall);
    void setBottomWall(bool wall);
    void setVal(int val);
    void print(void);

private:
    bool leftWall, rightWall, topWall, bottomWall;
    int value;
};

class Maze
{
public:
    void init(int rows, int cols);
    void setCell(int row, int col, Cell cell);
    Cell getCell(int row, int col);
    void print(void);

private:
    int rows, cols;
    Cell **maze;
};
