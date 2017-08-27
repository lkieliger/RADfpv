#define PLOT_OUTPUT
//#define VERBOSE_OUTPUT
int verboseOutputCount = 0;

#include <Servo.h>
#include <PID_v1.h>
#include "Motor.h"
#include "Gyroscope.h"
#include "Filters.h"
#include "AttitudeController.h"
#include "PIDController.h"

Gyroscope gyro;
MovingAverage movingAveragePitch{4};
MovingAverage movingAverageRoll{4};
LeakyIntegrator leakyIntegratorRoll{0.75};

AttitudeController attitude;
PIDController pid;

const float PIDConstantStep = 0.01;
AxisSelector sel = PITCH_SELECTOR;

void setup()
{
  gyro.init();
  attitude.init();
  pid.init();
  attitude.init();
}

// Increase the speed of the motor from low to high as set by the user
void Run()
{
  char currentChar = Serial.read();
  
  switch (currentChar){
    //------------------------------
    //  Thrust control
    //------------------------------
    case 'j':
      attitude.increaseBaseThrust();
      break;
    case 'k':
      attitude.decreaseBaseThrust();
      break;
    case 'l':
      attitude.shutdownMotors();
      break;
    case 'u':
      attitude.startupMotors();
      break;
    case 'i':
      attitude.resetControls();
      break;
      
    //------------------------------
    //  Calibration
    //------------------------------
    
    case 't':
      pid.updateSetpoints(gyro.getPitch(), gyro.getRoll());
      break;

    //------------------------------
    //  PID constant control
    //------------------------------

    case 'e':
      pid.updateKp(PIDConstantStep);
      break;
    case 'r':
      pid.updateKp(-PIDConstantStep);
      break;
    case 'd':
      pid.updateKi(PIDConstantStep);
      break;
    case 'f':
      pid.updateKi(-PIDConstantStep);
      break;
    case 'c':
      pid.updateKd(PIDConstantStep);
      break;
    case 'v':
      pid.updateKd(-PIDConstantStep);
      break;
    default:
      break;
  }
  stabilize();
  
  attitude.outputThrustSignal();
}


void stabilize(){

  float roll = gyro.getRoll();
  float pitch = gyro.getPitch();

  movingAveragePitch.addSample(pitch);
  leakyIntegratorRoll.addSample(roll);
  movingAverageRoll.addSample(roll);

  pid.updateInputs(pitch, leakyIntegratorRoll.getCurrentValue());
  pid.compute();

  attitude.rotatePitch(pid.getPitchOutput());
  attitude.rotateRoll(pid.getRollOutput());

  #ifdef VERBOSE_OUTPUT
  if(verboseOutputCount > 1000){
    verboseOutputCount = 0 ;
    Serial.print("Pitch: ");
    Serial.print(pitch, 4);
    Serial.print(" Roll: ");
    Serial.print(roll, 4);
    Serial.print(" YC: ");
    Serial.print(attitude.getYawControl());
    Serial.print(" PC: ");
    Serial.print(attitude.getPitchControl());
    Serial.print(" RC: ");
    Serial.print(attitude.getRollControl());
    Serial.print(" YPID: ");
    Serial.print("_");
    Serial.print(" PPID: ");
    Serial.print(pid.getPitchOutput(), 4);
    Serial.print(" R.PID: ");
    Serial.print(pid.getRollOutput(), 4);
    Serial.print(" Kp: ");
    Serial.print(pid.getKp(), 3);
    Serial.print(" Ki: ");
    Serial.print(pid.getKi(), 3);
    Serial.print(" Kd: ");
    Serial.println(pid.getKd(), 3);
  } else {
    verboseOutputCount += +1;
  }
  #endif

  #ifdef PLOT_OUTPUT
    Serial.print(roll, 4);
    Serial.print(" ");
    Serial.print(pid.getRollOutput());
    Serial.print(" ");
    Serial.print(movingAverageRoll.getCurrentValue(), 4);
    Serial.print(" ");
    Serial.println(leakyIntegratorRoll.getCurrentValue(), 4);
  #endif
}

void loop()
{
    // wait for MPU interrupt or extra packet(s) available
    while (!gyro.isReady()) {
        Run();
    }

    gyro.actualize();
}


