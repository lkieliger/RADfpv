#define PLOT_OUTPUT
//#define VERBOSE_OUTPUT
int verboseOutputCount = 0;

#include <Servo.h>
#include <PID_v1.h>
#include "Motor.h"
#include "Gyroscope.h"
#include "Filters.h"
#include "PIDController.h"

Gyroscope gyro;
MovingAverage movingAveragePitch{4};
MovingAverage movingAverageRoll{4};
LeakyIntegrator leakyIntegratorRoll{0.75};

PIDController pid;

typedef struct QuadControlDef
{
  int yaw;
  int pitch;
  int roll;
} QuadControlDef;

QuadControlDef quadControl;

constexpr int NUMMOTORS = 4;
Motor motors[NUMMOTORS] = {{5}, {6}, {10}, {11}};

// Stores the settings for all ESC. This can be made specific to each ESC, but that's not needed
// for a quadcopter project
typedef struct ESCSettingsDef
{
  int Low;
  int High;
  int Startup;
} ESCSettingsDef;

ESCSettingsDef ESCSettings; 

int currentGlobalSpeed;
int yawControl;
int pitchControl;
int rollControl;

const int ESC_HIGH_DEFAULT = 1600;
const int ESC_LOW_DEFAULT = 0;
const int ESC_STARTUP_DEFAULT = 1000;
const int ESC_STEP = 10;

const int DEFAULT_CONTROL_BOUND = 50;
const int MIN_YAW_CONTROL = -DEFAULT_CONTROL_BOUND;
const int MAX_YAW_CONTROL = DEFAULT_CONTROL_BOUND;
const int MIN_PITCH_CONTROL = -DEFAULT_CONTROL_BOUND;
const int MAX_PITCH_CONTROL = DEFAULT_CONTROL_BOUND;
const int MIN_ROLL_CONTROL = -DEFAULT_CONTROL_BOUND;
const int MAX_ROLL_CONTROL = DEFAULT_CONTROL_BOUND;

const float PIDConstantStep = 0.01;
AxisSelector sel = PITCH_SELECTOR;

void setup()
{
  gyro.init();
  pid.init();
  
  // Set the ESC settings to the defaults
  ESCSettings.Low   = ESC_LOW_DEFAULT;
  ESCSettings.High  = ESC_HIGH_DEFAULT;
  ESCSettings.Startup = ESC_STARTUP_DEFAULT;

  quadControl.yaw = 0;
  quadControl.pitch = 0;
  quadControl.roll = 0;

  // Send a low signal initially for normal mode
  for (int i = 0; i < NUMMOTORS; i++)
  {
    motors[i].init();
    motors[i].writeThrust();
  }

  currentGlobalSpeed = ESCSettings.Low;

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
      //Serial.println("\nIncreasing motor speed by step");
      if (currentGlobalSpeed + ESC_STEP < ESCSettings.High) {
        currentGlobalSpeed = currentGlobalSpeed + ESC_STEP;
        //Serial.println("New speed = ");
        //Serial.print(currentGlobalSpeed);
      }

      else
      {
        //Serial.println("\nMax speed reached\n");
      }
      break;
    case 'k':
      //Serial.println("\nDecreasing motor speed by step\n");
      if (currentGlobalSpeed - ESC_STEP >= ESCSettings.Low)
      {
        currentGlobalSpeed = currentGlobalSpeed - ESC_STEP;
        //Serial.println("New speed = ");
        //Serial.print(currentGlobalSpeed);
      }

      else
      {
        //Serial.println("\nMin speed reached\n");
      }
      break;
    case 'l':
      //Serial.println("\nStopping motors\n");
      currentGlobalSpeed = ESCSettings.Low;
      break;
    case 'u':
      //Serial.println("\nPlacing throttle at startup\n");
      currentGlobalSpeed = ESCSettings.Startup;
      break;
    case 'i':
      resetControls();
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
  applyMotorSpeed();
}

void applyMotorSpeed(){
  /**
   * Yaw is the vertical axis
   * Pitch is the lateral axis
   * Roll is the longitudinal axis
   */

  for (int i = 0; i < NUMMOTORS; i++)
  {
    motors[i].setThrust(currentGlobalSpeed);
  }
  
  if (quadControl.yaw != 0){
    /** 
     * Unbalance angular momentum of the propellers, the body of the drone pivots in the opposite direction to 
     * keep the total angular momentum unchanged. Increase thrust on the other motors to keep the total forward force
     * unchanged.
     */
    motors[1].updateThrust( - quadControl.yaw);
    motors[3].updateThrust( - quadControl.yaw);

    motors[0].updateThrust( + quadControl.yaw);
    motors[2].updateThrust( + quadControl.yaw);
  }

  if (quadControl.pitch != 0){
    motors[0].updateThrust( + quadControl.pitch);
    motors[1].updateThrust( + quadControl.pitch);

    motors[2].updateThrust( - quadControl.pitch);
    motors[3].updateThrust( - quadControl.pitch);
  }

  if (quadControl.roll != 0){
    motors[0].updateThrust( + quadControl.roll);
    motors[3].updateThrust( + quadControl.roll);

    motors[1].updateThrust( - quadControl.roll);
    motors[2].updateThrust( - quadControl.roll);
  }
  
  for (int i = 0; i < NUMMOTORS; i++)
  {
    motors[i].writeThrust();
  }  
}

void rotateYaw(int d){
  quadControl.yaw = constrain(d, MIN_YAW_CONTROL, MAX_YAW_CONTROL);
}

void rotatePitch(int d){
  quadControl.pitch = constrain(d, MIN_PITCH_CONTROL, MAX_PITCH_CONTROL);
}

void rotateRoll(int d){
  quadControl.roll = constrain(d, MIN_ROLL_CONTROL, MAX_ROLL_CONTROL);
}

void resetControls(){
  quadControl.yaw = 0;
  quadControl.pitch = 0;
  quadControl.roll = 0;
}

void stabilize(){

  float roll = gyro.getRoll();
  float pitch = gyro.getPitch();

  movingAveragePitch.addSample(pitch);
  leakyIntegratorRoll.addSample(roll);
  movingAverageRoll.addSample(roll);

  pid.updateInputs(pitch, leakyIntegratorRoll.getCurrentValue());
  pid.compute();

  rotatePitch(pid.getPitchOutput());
  rotateRoll(pid.getRollOutput());

  #ifdef VERBOSE_OUTPUT
  if(verboseOutputCount > 1000){
    verboseOutputCount = 0 ;
    Serial.print("Pitch: ");
    Serial.print(pitch, 4);
    Serial.print(" Roll: ");
    Serial.print(roll, 4);
    Serial.print(" YC: ");
    Serial.print(quadControl.yaw);
    Serial.print(" PC: ");
    Serial.print(quadControl.pitch);
    Serial.print(" RC: ");
    Serial.print(quadControl.roll);
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


