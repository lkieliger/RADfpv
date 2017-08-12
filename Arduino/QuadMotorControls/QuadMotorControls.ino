#include <Servo.h>

typedef struct QuadControlDef
{
  int yaw;
  int pitch;
  int roll;
};

QuadControlDef quadControl;

#define NUMMOTORS 4
typedef struct MotorDef
{
    Servo   motor; 
    int     pin;   // Indicates the pin this motor is connected to
    int     thrust; 
};

MotorDef motors[NUMMOTORS];

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

const int DEFAULT_CONTROL_BOUND = 3;
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
  
  // Attach motors to pins
  motors[0].pin =  5;
  motors[1].pin =  6;
  motors[2].pin =  10;
  motors[3].pin =  11;

  // Attach motors to their corresponding pins
  for(int i = 0; i < NUMMOTORS; i++)
  {
    motors[i].motor.attach(motors[i].pin);
  }

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
    motors[i].motor.write(ESCSettings.Low);
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
    if (currentChar == 'h')
    {
      Serial.println("\nHigh\n");
      currentGlobalSpeed = ESCSettings.High;
    }
    if (currentChar == 'j')
    {
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
    }
    if (currentChar == 'k')
    {
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
    }
    if (currentChar == 'l')
    {
      Serial.println("\nStopping motors\n");
      currentGlobalSpeed = ESCSettings.Low;
    }
    if (currentChar == 'u')
    {
      Serial.println("\nPlacing throttle at startup\n");
      currentGlobalSpeed = ESCSettings.Startup;
    }
    applyMotorSpeed();
  }
}

void applyMotorSpeed(){

  for (int i = 0; i < NUMMOTORS; i++)
  {
    motors[i].thrust = currentGlobalSpeed;
  }
  
  if (quadControl.yaw != 0){
    /* Unbalance angular momentum of the propellers, the body of the drone pivots in the opposite direction to 
     * keep the total angular momentum unchanged. Increase thrust on the other motors to keep the total forward force
     * unchanged.
     */
    motors[1].thrust -= quadControl.yaw;
    motors[3].thrust -= quadControl.yaw;

    motors[0].thrust += quadControl.yaw;
    motors[2].thrust += quadControl.yaw;
  }

  if (quadControl.pitch != 0){
    
  }

  if (quadControl.roll != 0){
    
  }
  
  for (int i = 0; i < NUMMOTORS; i++)
  {
    motors[i].motor.write(motors[i].thrust);
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

void loop()
{
  Run();
  while(1) { } 
}


