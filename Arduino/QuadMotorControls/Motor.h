#ifndef MOTOR_H
#define MOTOR_H

#include <Servo.h>

class Motor{
public:
  Motor(int outputPin) : thrust(0), pin(outputPin), servoControl{}
  {
  }

  void init()
  {
    servoControl.attach(pin);
  }

  void setThrust(int t)
  {
    thrust = constrain(t, MIN_THRUST, MAX_THRUST);
  }

  void updateThrust(int delta)
  {
    thrust += delta;
    thrust = constrain(thrust, MIN_THRUST, MAX_THRUST);
  }

  void writeThrust()
  {
    servoControl.writeMicroseconds(thrust);
  }

  void printStatus()
  {
    Serial.print("MP");
    Serial.print(pin);
    
    Serial.print(". Thrust set to ");
    Serial.println(thrust);
  }
  
private:
  Servo servoControl;
  int thrust;
  int pin;
  const int MIN_THRUST = 0;
  const int MAX_THRUST = 1500;
};

#endif
