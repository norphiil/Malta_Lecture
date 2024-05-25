#include "devices.h"
#include "FastLED.h"
#include "device_gyro_accel.h"

#define LINE_FOLLOW_SPEED 50
#define TURN_SPEED 50
#define TURN_DURATION 200

#define LED_PIN 4
#define NUM_LEDS 1
CRGB leds[NUM_LEDS]; // Define the LED array
bool atIntersection = false;

// Initialize objects
Motor motor;
Servo servo;
IR ir;
Ultrasonic ultrasonic;
GyroAccel gyroAccel;
bool stop = false;
float actualYaw = 0;
bool first = true;
bool isTurningcorner = false;

Direction lastDirection = FORWARDS; // Store the last known direction

void flashLed()
{
  leds[0] = CRGB::HotPink;
  FastLED.show(); // Update the LED strip with the new color
  delay(500);     // Wait for 500 milliseconds

  // Turn off the LED
  leds[0] = CRGB::Black;
  FastLED.show(); // Update the LED strip
}

void setup()
{
  Serial.begin(9600);
  motor.init();
  servo.init();
  ultrasonic.init();
  ir.init();
  gyroAccel.init();
  delay(2000);
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(255);

  // Turn the robot at angle 90
  // motor.move(RIGHT, TURN_SPEED);
  // uint16_t targetAngle = 90;
  // bool stop_turn = false;

  // while (!stop_turn)
  // {
  //   float roll, pitch, yaw;
  //   gyroAccel.getRotation(&roll, &pitch, &yaw);
  //   Serial.println(yaw);
  //   Serial.println(targetAngle);
  //   motor.move(RIGHT, TURN_SPEED);
  //   if (gyroAccel.getAnglesDiff(yaw, actualYaw) > 90)
  //   {
  //     motor.stop();
  //   }
  //   else
  //   {
  //     motor.move(RIGHT, TURN_SPEED);
  //   }
  // }
}

void loop()
{
  if (stop)
    return;
  // Read IR sensors
  bool leftSensor = ir.get(IR_LEFT);
  bool middleSensor = ir.get(IR_MIDDLE);
  bool rightSensor = ir.get(IR_RIGHT);

  float roll, pitch, yaw;
  gyroAccel.getRotation(&roll, &pitch, &yaw);
  if (first)
  {
    actualYaw = yaw;
    first = false;
  }
  // If all sensors detect a line, move forward
  if (leftSensor && middleSensor && rightSensor)
  {
    if (atIntersection)
    {
      motor.move(lastDirection, LINE_FOLLOW_SPEED);
      delay(TURN_DURATION);
    }
    atIntersection = true;
    motor.stop();
    lastDirection = FORWARDS;
    servo.setAngle(90);
    unsigned int distance_F = ultrasonic.get();
    flashLed();
    servo.setAngle(180);
    unsigned int distance_L = ultrasonic.get();
    flashLed();
    servo.setAngle(0);
    unsigned int distance_R = ultrasonic.get();
    flashLed();
    if (distance_L > distance_F && distance_L > distance_R)
    {
      motor.move(LEFT, LINE_FOLLOW_SPEED);
      delay(TURN_DURATION);
      while (!ir.get(IR_MIDDLE))
      {
        motor.move(LEFT, LINE_FOLLOW_SPEED);
        delay(50);
      }
    }
    else if (distance_R > distance_F && distance_R > distance_L)
    {
      motor.move(RIGHT, LINE_FOLLOW_SPEED);
      delay(TURN_DURATION);
      while (!ir.get(IR_MIDDLE))
      {
        motor.move(RIGHT, LINE_FOLLOW_SPEED);
        delay(50);
      }
    }
    else
    {
      motor.move(FORWARDS, LINE_FOLLOW_SPEED);
    }
    servo.setAngle(90);
  }
  else
  {
    atIntersection = false;
    // If only the left sensor detects a line, turn left
    if (leftSensor)
    {
      motor.move(FORWARDS_LEFT, LINE_FOLLOW_SPEED);
      lastDirection = FORWARDS_LEFT;
    }
    // If only the right sensor detects a line, turn right
    else if (rightSensor)
    {
      motor.move(FORWARDS_RIGHT, LINE_FOLLOW_SPEED);
      lastDirection = FORWARDS_RIGHT;
    }
    // If only the middle sensor detects a line, move forward
    else if (middleSensor)
    {
      motor.move(FORWARDS, LINE_FOLLOW_SPEED);
      lastDirection = FORWARDS;
    }
    else
    {
      if (lastDirection == FORWARDS)
      { // This should be the end of the line, so stop
        motor.stop();
        stop = true;
        return;
      }
      if (lastDirection == FORWARDS_LEFT || lastDirection == LEFT)
      { // If it last turned left or right, then assume we should continue turning
        // Perform a right turn for TURN_DURATION milliseconds
        bool stop_turn = false;
        while (!stop_turn)
        {
          motor.move(LEFT, TURN_SPEED);
          if (gyroAccel.getAnglesDiff(yaw, actualYaw) > 30)
          {
            isTurningcorner = true;
          }
          if (ir.get(IR_MIDDLE))
          {
            if (isTurningcorner && gyroAccel.getAnglesDiff(yaw, actualYaw) > 90)
            {
              motor.stop();
              stop_turn = true;
              isTurningcorner = false;
              actualYaw = yaw;
            }
            else if (!isTurningcorner)
            {
              stop_turn = true;
            }
          }
        }
      }
      else
      {
        // Perform a left turn for TURN_DURATION milliseconds
        // while (!ir.get(IR_MIDDLE))
        //   motor.move(RIGHT, TURN_SPEED);
        bool stop_turn = false;
        while (!stop_turn)
        {
          motor.move(RIGHT, TURN_SPEED);
          if (gyroAccel.getAnglesDiff(yaw, actualYaw) > 30)
          {
            isTurningcorner = true;
          }
          if (ir.get(IR_MIDDLE))
          {
            if (isTurningcorner && gyroAccel.getAnglesDiff(yaw, actualYaw) > 90)
            {
              motor.stop();
              stop_turn = true;
              isTurningcorner = false;
              actualYaw = yaw;
            }
            else if (!isTurningcorner)
            {
              stop_turn = true;
            }
          }
        }
      }
    }
  }

  delay(100); // Adjust delay as needed
}