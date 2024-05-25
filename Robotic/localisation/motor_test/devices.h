#include <Arduino.h>

// class PID
// {
// public:
//     void calibrate(void);

// private:
//     float Kp = 0; // related to the proportional control term;
//                   // change the  value by trial-and-error (ex: 0.07).
//     float Ki = 0; // related to the integral  control term;
//                   // change the value by trial-and-error (ex: 0.0008).
//     float Kd = 0; // related to the derivative control term;
//                   // change the  value by trial-and-error (ex: 0.6).
//     int P;
//     int I;
//     int D;
// };

class Pos
{
public:
    void init(double x, double y);
    double getX(void);
    double getY(void);
    void set(double x, double y);
    double distanceTo(Pos pos);
    double calculateTargetAngle(Pos pos);
    void toString(void);

private:
    double x,
        y;
};

#include "device_servo.h"
#include "device_ir.h"
#include "device_ultrasonic.h"
#include "device_gyro_accel.h"
#include "device_motor.h"

class Path
{
public:
    void init(Pos path_list[100], uint8_t number_of_points);
    void run(Motor motor, uint8_t speed, uint8_t number_of_points);

private:
    Pos path_list[100];
};
