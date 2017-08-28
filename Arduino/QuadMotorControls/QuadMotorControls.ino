//#define PLOT_OUTPUT
#define VERBOSE_OUTPUT
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

void loop()
{
    // wait for MPU interrupt or extra packet(s) available
    while (!gyro.isReady()) {
        Run();
    }

    gyro.actualize();
}

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

    case 's':
      pid.swapPIDSettingAxis();
      break;
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
  printStatus();
  attitude.outputThrustSignal();
}


void stabilize(){

  float roll = gyro.getRoll();
  float pitch = gyro.getPitch();

  // ----------- SAFEGUARD ----------- //
  if( abs(roll) > 50.0 || abs(pitch) > 50.0){
    attitude.emergencyBrake();
    pid.disable();
  }

  if( attitude.getBaseThrust() <= AttitudeController::STARTUP_THRUST){
    pid.disable();
  } else {
    pid.enable();
  }

  movingAveragePitch.addSample(pitch);
  leakyIntegratorRoll.addSample(roll);
  movingAverageRoll.addSample(roll);

  pid.updateInputs(pitch, leakyIntegratorRoll.getCurrentValue());
  pid.compute();

  attitude.rotatePitch(pid.getPitchOutput());
  attitude.rotateRoll(pid.getRollOutput());
}

void printStatus(){
  #ifdef VERBOSE_OUTPUT
  if(verboseOutputCount > 1000){
    verboseOutputCount = 0 ;
    Serial.print("Base thrust: ");
    Serial.print(attitude.getBaseThrust());
    Serial.print(" Front left: ");
    Serial.print(attitude.getMotorThrust(AttitudeController::FRONT_LEFT));
    Serial.print(" Front right: ");
    Serial.print(attitude.getMotorThrust(AttitudeController::FRONT_RIGHT));
    Serial.print(" Back right: ");
    Serial.print(attitude.getMotorThrust(AttitudeController::BACK_RIGHT));
    Serial.print(" Back left: ");
    Serial.println(attitude.getMotorThrust(AttitudeController::BACK_LEFT));
    
    Serial.print("Pitch: ");
    Serial.print(gyro.getPitch(), 4);
    Serial.print(" Roll: ");
    Serial.print(gyro.getRoll(), 4);
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
  } 
  verboseOutputCount += +1;
  
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


