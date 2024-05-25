#include <Elegoo.h>

#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_BIN_1 8
#define PIN_Motor_AIN_1 7
#define PIN_Motor_STBY 3

#define speed_Max 255
#define direction_just true
#define direction_back false
#define direction_void 3

#define Duration_enable true
#define Duration_disable false
#define control_enable true
#define control_disable false

class Motor
{
public:
  void init()
  {
    pinMode(PIN_Motor_PWMA, OUTPUT);
    pinMode(PIN_Motor_PWMB, OUTPUT);
    pinMode(PIN_Motor_AIN_1, OUTPUT);
    pinMode(PIN_Motor_BIN_1, OUTPUT);
    pinMode(PIN_Motor_STBY, OUTPUT);
  }
  void leftMotor(uint8_t speed, uint8_t direction)
  {
    digitalWrite(PIN_Motor_BIN_1, direction);
    analogWrite(PIN_Motor_PWMB, speed);
  }

  void rightMotor(uint8_t speed, uint8_t direction)
  {
    digitalWrite(PIN_Motor_AIN_1, direction);
    analogWrite(PIN_Motor_PWMA, speed);
  }

  void setSTBY(uint8_t direction)
  {
    digitalWrite(PIN_Motor_STBY, direction);
  }

  void goForward(uint8_t speed)
  {
    setSTBY(HIGH);
    leftMotor(speed, HIGH);
    rightMotor(speed, HIGH);
  }

  void goBackward(uint8_t speed)
  {
    setSTBY(HIGH);
    leftMotor(speed, LOW);
    rightMotor(speed, LOW);
  }

  void goLeft(uint8_t speed)
  {
    setSTBY(HIGH);
    leftMotor(speed, HIGH);
    rightMotor(speed, LOW);
  }

  void goRight(uint8_t speed)
  {
    setSTBY(HIGH);
    leftMotor(speed, LOW);
    rightMotor(speed, HIGH);
  }

  void goLeftForward(uint8_t speed)
  {
    setSTBY(HIGH);
    leftMotor(speed / 2, HIGH);
    rightMotor(speed, HIGH);
  }

  void goLeftBackward(uint8_t speed)
  {
    setSTBY(HIGH);
    leftMotor(speed / 2, LOW);
    rightMotor(speed, LOW);
  }

  void goRightForward(uint8_t speed)
  {
    setSTBY(HIGH);
    leftMotor(speed, HIGH);
    rightMotor(speed / 2, HIGH);
  }

  void goRightBackward(uint8_t speed)
  {
    setSTBY(HIGH);
    leftMotor(speed, LOW);
    rightMotor(speed / 2, LOW);
  }

  void stop_it()
  {
    analogWrite(PIN_Motor_PWMA, 0);
    analogWrite(PIN_Motor_PWMB, 0);
    setSTBY(LOW);
  }
};

Motor motor;

void setup()
{
  motor.init();
  delay(1000);

  // motor.goForward(100);
  // delay(5000);
  // motor.stop_it();
  // delay(1000);
  // motor.goBackward(100);
  // delay(5000);
  // motor.stop();
}

void loop()
{
}