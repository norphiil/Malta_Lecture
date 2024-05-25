#include "devices.h"

// Change to 0 or comment this line out to switch off debug mode and hide Serial prints
#define DEBUG_MODE 1

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////      TARGET      //////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Initialises the position
 */
void Pos::init(double x, double y)
{
    this->x = x;
    this->y = y;
}

/**
 * Returns the X coordinate of the position
 * @return The X coordinate
 */
double Pos::getX()
{
    return this->x;
}

/**
 * Returns the Y coordinate of the position
 * @return The Y coordinate
 */
double Pos::getY()
{
    return this->y;
}

/**
 * Sets the X and Y coordinates of the position
 * @param x The new X coordinate
 * @param y The new Y coordinate
 */
void Pos::set(double x, double y)
{
    this->x = x;
    this->y = y;
}

/**
 * Returns the distance to another position
 * @param pos The other position
 * @return The distance to the other position
 */
double Pos::distanceTo(Pos pos)
{
    double x_diff = pos.getX() - this->x;
    double y_diff = pos.getY() - this->y;
    return sqrt(x_diff * x_diff + y_diff * y_diff);
}

/**
 * Prints the position to the serial monitor
 */
void Pos::toString()
{
    Serial.print("X=");
    Serial.print(this->x);
    Serial.print("| Y=");
    Serial.println(this->y);
}

/**
 * Calculates the angle to another position
 * @param pos The other position
 * @return The angle to the other position
 */
double Pos::calculateTargetAngle(Pos pos)
{
    double x_diff = pos.getX() - this->x;
    double y_diff = pos.getY() - this->y;
    return atan2(y_diff, x_diff) * 180 / PI;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////      PATH      ////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Initialises the path
 * @param path_list The list of positions in the path
 */
void Path::init(Pos path_list[100], uint8_t number_of_points)
{
    for (int i = 0; i < number_of_points; i++)
    {
        this->path_list[i] = *(path_list + i);
    }
}

/**
 * Runs the path
 * @param motor The motor object to use
 * @param speed The speed at which to move
 */
void Path::run(Motor motor, uint8_t speed, uint8_t number_of_points)
{
    for (int i = 0; i < number_of_points; i++)
    {
        if (this->path_list[i].getX() != this->path_list[i].getX() || this->path_list[i].getY() != this->path_list[i].getY())
        {
            break;
        }
        Pos current_pos = Pos();
        current_pos.init(0, 0);

        motor.goToPoint(current_pos, this->path_list[i], speed);
    }

    Serial.println("Path completed!");
}

// //////////////////////////////////////////////////////////////////////////////////////////////////////////
// //////////////////////////////////////////      SIMPLEPID      ////////////////////////////////////////////
// //////////////////////////////////////////////////////////////////////////////////////////////////////////

// void PID::calibrate()
// {
//     //   uint16_t position = qtr.readLineBlack(sensorValues); //read the current position
//     int error = 3500 - position; // 3500 is the ideal position  (the centre)

//     P = error;
//     I = I + error;
//     D = error - lastError;

//     lastError = error;
//     int motorspeed = P * Kp + I * Ki + D * Kd; // calculate the correction
//                                                // needed to be applied to the speed

//     int motorspeeda = basespeeda + motorspeed;
//     int motorspeedb = basespeedb - motorspeed;

//     if (motorspeeda > maxspeeda)
//     {
//         motorspeeda = maxspeeda;
//     }
//     if (motorspeedb > maxspeedb)
//     {
//         motorspeedb = maxspeedb;
//     }

//     if (motorspeeda < 0)
//     {
//         motorspeeda = 0;
//     }
//     if (motorspeedb < 0)
//     {
//         motorspeedb = 0;
//     }
//     forward_brake(motorspeeda, motorspeedb);
// }