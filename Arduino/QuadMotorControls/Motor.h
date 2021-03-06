#ifndef MOTOR_H
#define MOTOR_H

#include <Servo.h>

class Motor{
public:
  Motor(int outputPin) : servoControl{}, thrust(0), pin(outputPin)
  {
  }

  void init()
  {
    servoControl.attach(pin);
  }

  void setThrust(short t)
  {
    thrust = constrain(t, MIN_THRUST, MAX_THRUST);
  }

  void updateThrust(short delta)
  {
    thrust += delta;
    thrust = constrain(thrust, MIN_THRUST, MAX_THRUST);
  }

  /**
   * This function actually applies the thrust to the physical motors.
   * The reasing it is separed to update thrust is because there can be multiple updates
   * to the thrust value in order to reach the desired thrust for stabilization.
   */
  void writeThrust()
  {
    servoControl.writeMicroseconds(thrust);
  }

  short getThrust(){
    return thrust;
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
  short thrust;
  short pin;
  const short MIN_THRUST = 0;
  const short MAX_THRUST = 1400;
};

#endif
