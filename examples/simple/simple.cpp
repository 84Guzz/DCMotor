#include <Arduino.h>
#include <DCMotor.h>

DCMotor motor;
DCMotorParam param;

void setup()
{
  param.mode = DC_MOTOR_TWO_DIR;
  param.ramp = 1000;
  param.startStopDelay = 2000;
  param.enbPin = 7;
  param.pwmPin = 9;

  motor.begin(param);
  motor.setSpeed(200);
  motor.cmdA();
  Serial.begin(9600);
}

void loop()
{
  motor.update();
  switch (motor.getState())
  {
  case DC_MOTOR_RUNNING_A:
    motor.cmdB();
    break;
  case DC_MOTOR_RUNNING_B:
    motor.cmdA();
    break;
  }

  if (motor.getSpeed() > 0)
  {
    Serial.println("speed:" + String(motor.getSpeed()));
  }
  delay(100);
}