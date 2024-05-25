#include "devices.h"

// Initialize objects
Motor motor;
int cell_width = 30; // 30x30 cm
Maze maze;
int staticForwardDistance;
unsigned long lastTime;

void setup()
{
  Serial.begin(9600);
  motor.init();
  delay(2000);
  maze.init(6, 6);

  lastTime = millis();
  motor.getServo().setAngle(90);
  int forwardDistance = motor.getUltrasonic().get();
  staticForwardDistance = forwardDistance;

  motor.getServo().setAngle(0);
  int leftDistance = motor.getUltrasonic().get();

  motor.getServo().setAngle(180);
  int rightDistance = motor.getUltrasonic().get();

  motor.getServo().setAngle(90);

  Cell forwardCell = maze.getCell(ceil(forwardDistance / cell_width), 0);
  forwardCell.setTopWall(true);
  maze.setCell(ceil(forwardDistance / cell_width), 0, forwardCell);

  Cell leftCell = maze.getCell(0, ceil(leftDistance / cell_width));
  leftCell.setLeftWall(true);
  maze.setCell(0, ceil(leftDistance / cell_width), leftCell);

  Cell rightCell = maze.getCell(0, ceil(rightDistance / cell_width));
  rightCell.setRightWall(true);
  maze.setCell(0, ceil(rightDistance / cell_width), rightCell);
}

void loop()
{
  unsigned long currentTime = millis();
  if (currentTime - motor.getLastTime() > 100)
  {
    motor.setLastTime(currentTime);
    int forwardDistance = motor.getUltrasonic().get();
    int leftDistance = motor.getUltrasonic().get();
    int rightDistance = motor.getUltrasonic().get();

    if (forwardDistance < staticForwardDistance)
    {
      motor.forwards(255);
    }
    else
    {
      motor.right(255);
    }
  }
}