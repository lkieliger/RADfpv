#include <Servo.h>
#include "Motor.h"

typedef struct QuadControlDef
{
  int yaw;
  int pitch;
  int roll;
};

QuadControlDef quadControl;

#define NUMMOTORS 4
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

const int ESC_HIGH_DEFAULT = 180;
const int ESC_LOW_DEFAULT = 0;
const int ESC_STARTUP_DEFAULT = 45;
const int ESC_STEP = 1;

const int DEFAULT_CONTROL_BOUND = 10;
const int MIN_YAW_CONTROL = -DEFAULT_CONTROL_BOUND;
const int MAX_YAW_CONTROL = DEFAULT_CONTROL_BOUND;
const int MIN_PITCH_CONTROL = -DEFAULT_CONTROL_BOUND;
const int MAX_PITCH_CONTROL = DEFAULT_CONTROL_BOUND;
const int MIN_ROLL_CONTROL = -DEFAULT_CONTROL_BOUND;
const int MAX_ROLL_CONTROL = DEFAULT_CONTROL_BOUND;

void setup()
{
  
  // Required for I/O from Serial monitor
  Serial.begin(9600);
  Serial.println("Setup: Serial port communication at 9600bps");

  // Set the ESC settings to the defaults
  ESCSettings.Low   = ESC_LOW_DEFAULT;
  ESCSettings.High  = ESC_HIGH_DEFAULT;
  ESCSettings.Startup = ESC_STARTUP_DEFAULT;

  quadControl.yaw = 0;
  quadControl.pitch = 0;
  quadControl.roll = 0;
  
}

// Increase the speed of the motor from low to high as set by the user
void Run()
{
  // Send a low signal initially for normal mode
  for (int i = 0; i < NUMMOTORS; i++)
  {
    motors[i].init();
    motors[i].writeThrust();
    motors[i].printStatus();
  }
  Serial.println("Running ESC");
  Serial.println("Step = ");
  Serial.print(ESC_STEP);
  Serial.println("\nPress 'u' to increase speed, 'd' to reduce speed");

  currentGlobalSpeed = ESCSettings.Low;
  while (1) {
    while (!Serial.available())
    {
    }
    char currentChar = Serial.read();

    switch (currentChar){
      case 'h':
          Serial.println("\nHigh\n");
          currentGlobalSpeed = ESCSettings.High;
        break;
      case 'j':
        Serial.println("\nIncreasing motor speed by step");
        if (currentGlobalSpeed + ESC_STEP < ESCSettings.High) {
          currentGlobalSpeed = currentGlobalSpeed + ESC_STEP;
          Serial.println("New speed = ");
          Serial.print(currentGlobalSpeed);
        }
  
        else
        {
          Serial.println("\nMax speed reached\n");
        }
        break;
      case 'k':
        Serial.println("\nDecreasing motor speed by step\n");
        if (currentGlobalSpeed - ESC_STEP >= ESCSettings.Low)
        {
          currentGlobalSpeed = currentGlobalSpeed - ESC_STEP;
          Serial.println("New speed = ");
          Serial.print(currentGlobalSpeed);
        }
  
        else
        {
          Serial.println("\nMin speed reached\n");
        }
        break;
      case 'l':
        Serial.println("\nStopping motors\n");
        currentGlobalSpeed = ESCSettings.Low;
        break;
      case 'u':
        Serial.println("\nPlacing throttle at startup\n");
        currentGlobalSpeed = ESCSettings.Startup;
        break;
      case 'i':
        resetControls();
        break;
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
      default:
        break;
    }
    applyMotorSpeed();
  }
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
    motors[i].printStatus();
  }  
}

void rotateYaw(int d){
  quadControl.yaw = constrain(quadControl.yaw + d, MIN_YAW_CONTROL, MAX_YAW_CONTROL);
}

void rotatePitch(int d){
  quadControl.pitch = constrain(quadControl.pitch + d, MIN_PITCH_CONTROL, MAX_PITCH_CONTROL);
}

void rotateRoll(int d){
  quadControl.roll = constrain(quadControl.roll + d, MIN_ROLL_CONTROL, MAX_ROLL_CONTROL);
}

void resetControls(){
  quadControl.yaw = 0;
  quadControl.pitch = 0;
  quadControl.roll = 0;
}
void loop()
{
  Run();
  while(1) { } 
}


