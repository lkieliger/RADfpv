#include <Servo.h>
#include <PID_v1.h>
#include "Motor.h"
#include "Gyroscope.h"
//#include "PIDController.h"

Gyroscope gyro;

// Variables for the PID algorithm
constexpr float PIDConstantStep = 0.01;
double pitchSetpoint, pitchInput, pitchControlOutput;
double rollSetpoint, rollInput, rollControlOutput;
double Kp=0.50, Ki=0.15, Kd=0.05;
PID pitchPID(&pitchInput, &pitchControlOutput, &pitchSetpoint, Kp, Ki, Kd, DIRECT);
PID rollPID(&rollInput, &rollControlOutput, &rollSetpoint, Kp, Ki, Kd, REVERSE);

typedef struct QuadControlDef
{
  int yaw;
  int pitch;
  int roll;
};

QuadControlDef quadControl;

constexpr int NUMMOTORS = 4;
const Motor motors[NUMMOTORS] = {{5}, {6}, {10}, {11}};

// Stores the settings for all ESC. This can be made specific to each ESC, but that's not needed
// for a quadcopter project
typedef struct ESCSettingsDef
{
  int Low;
  int High;
  int Startup;
};

ESCSettingsDef ESCSettings; 

#define NORMAL_MODE

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

void setup()
{
  gyro.init();
  
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

  //pitchPID.SetSampleTime(50);
  pitchPID.SetOutputLimits(MIN_PITCH_CONTROL, MAX_PITCH_CONTROL);
  //rollPID.SetSampleTime(50);
  rollPID.SetOutputLimits(MIN_ROLL_CONTROL, MAX_ROLL_CONTROL);
}

// Increase the speed of the motor from low to high as set by the user
void Run()
{
  char currentChar = Serial.read();
  
  switch (currentChar){
    //------------------------------
    //  Thrust control
    //------------------------------
    case 'h':
        //Serial.println("\nHigh\n");
        currentGlobalSpeed = ESCSettings.High;
      break;
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
      gyro.calibrateGyro();

      pitchSetpoint = gyro.getPitch();
      rollSetpoint = gyro.getRoll();
    
      if(pitchPID.GetMode() == MANUAL || rollPID.GetMode() == MANUAL){
        pitchPID.SetMode(AUTOMATIC);
        rollPID.SetMode(AUTOMATIC);
      }
      break;
      
    //------------------------------
    //  Yaw pitch and roll control
    //------------------------------
    
    case 'q':
      rotateYaw(1);
      break;
    case 'w':
      rotateYaw(-1);
      break;
    case 'a':
      rotatePitch(1);
      break;
    case 's':
      rotatePitch(-1);
      break;
    case 'y':
      rotateRoll(1);
      break;
    case 'x':
      rotateRoll(-1);
      break;

    //------------------------------
    //  PID constant control
    //------------------------------

    case 'e':
      Kp += PIDConstantStep;
      updatePIDTunings();
      break;
    case 'r':
      Kp -= PIDConstantStep;
      updatePIDTunings();
      break;
    case 'd':
      Ki += PIDConstantStep;
      updatePIDTunings();
      break;
    case 'f':
      Ki -= PIDConstantStep;
      updatePIDTunings();
      break;
    case 'c':
      Kd += PIDConstantStep;
      updatePIDTunings();
      break;
    case 'v':
      Kd -= PIDConstantStep;
      updatePIDTunings();
      break;
      
    default:
      break;
  }
  stabilize();
  applyMotorSpeed();
}

void updatePIDTunings(){
  pitchPID.SetTunings(Kp, Ki, Kd);
  rollPID.SetTunings(Kp, Ki, Kd);
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

  rollInput = gyro.getRoll();
  pitchInput = gyro.getPitch();

  //pitchPID.Compute();
  rollPID.Compute();

  rotateRoll(rollControlOutput);
  rotatePitch(pitchControlOutput);
  
  Serial.print("Pitch: ");
  Serial.print(gyro.getPitch());
  Serial.print(" Roll: ");
  Serial.print(gyro.getRoll());
  Serial.print(" YC: ");
  Serial.print(quadControl.yaw);
  Serial.print(" PC: ");
  Serial.print(quadControl.pitch);
  Serial.print(" RC: ");
  Serial.print(quadControl.roll);
  Serial.print(" YPID: ");
  Serial.print("_");
  Serial.print(" PPID: ");
  Serial.print(pitchControlOutput);
  Serial.print(" R.PID: ");
  Serial.print(rollControlOutput);
  Serial.print(" Kp: ");
  Serial.print(Kp);
  Serial.print(" Ki: ");
  Serial.print(Ki);
  Serial.print(" Kd: ");
  Serial.println(Kd);
}

void loop()
{
    // wait for MPU interrupt or extra packet(s) available
    while (!gyro.isReady()) {
        Run();
    }
    
    gyro.actualize();
}


