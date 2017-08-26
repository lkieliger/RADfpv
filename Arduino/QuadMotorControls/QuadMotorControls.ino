#define PLOT_OUTPUT
//#define VERBOSE_OUTPUT


#include <Servo.h>
#include <PID_v1.h>
#include "Motor.h"
#include "I2Cdev.h"
#include "MovingAverage.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Variables for the PID algorithm
constexpr float PIDConstantStep = 0.01;
double pitchSetpoint, pitchInput, pitchControlOutput;
double rollSetpoint, rollInput, rollControlOutput;
double Kp=0.430, Ki=0.020, Kd=0.030;
PID pitchPID(&pitchInput, &pitchControlOutput, &pitchSetpoint, Kp, Ki, Kd, DIRECT);
PID rollPID(&rollInput, &rollControlOutput, &rollSetpoint, Kp, Ki, Kd, REVERSE);

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float desiredYpr[3]{0,0,0};

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

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

MovingAverage movingAveragePitch{5};
MovingAverage movingAverageRoll{5};

void setup()
{
  
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(230400);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

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

  pitchPID.SetSampleTime(1);
  pitchPID.SetOutputLimits(MIN_PITCH_CONTROL, MAX_PITCH_CONTROL);
  rollPID.SetSampleTime(1);
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
      calibrateGyro();
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
  // Inverse pitch and roll due to sensor mounting direction on quadcopter
  float pitch = ypr[1] * 180/M_PI;
  float roll = ypr[2] * 180/M_PI;

  float deltaPitch = desiredYpr[1] - pitch;
  float deltaRoll = desiredYpr[2] - roll;

  movingAveragePitch.append(pitch);
  movingAverageRoll.append(roll);

  rollInput = roll;
  pitchInput = pitch;

  //pitchPID.Compute();
  rollPID.Compute();

  rotateRoll(rollControlOutput);
  rotatePitch(pitchControlOutput);

  #ifdef VERBOSE_OUTPUT
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
    Serial.print(pitchControlOutput, 4);
    Serial.print(" R.PID: ");
    Serial.print(rollControlOutput, 4);
    Serial.print(" Kp: ");
    Serial.print(Kp, 3);
    Serial.print(" Ki: ");
    Serial.print(Ki, 3);
    Serial.print(" Kd: ");
    Serial.println(Kd, 3);
  #endif

  #ifdef PLOT_OUTPUT
    Serial.print(pitch, 4);
    Serial.print(" ");
    Serial.print(quadControl.pitch, 4);
    Serial.print(" ");
    Serial.print(movingAveragePitch.value(), 4);
    Serial.print(" ");
    Serial.print(roll, 4);
    Serial.print(" ");
    Serial.print(quadControl.roll, 4);
    Serial.print(" ");
    Serial.println(movingAverageRoll.value(), 4);
  #endif
}

void calibrateGyro(){
  float pitch = ypr[1] * 180/M_PI;
  float roll = ypr[2] * 180/M_PI;

  //desiredYpr[0] = 0.0;
  //desiredYpr[1] = pitch;
  //desiredYpr[2] = roll;
  
  pitchSetpoint = pitch;
  rollSetpoint = roll;

  if(pitchPID.GetMode() == MANUAL || rollPID.GetMode() == MANUAL){
    pitchPID.SetMode(AUTOMATIC);
    rollPID.SetMode(AUTOMATIC);
  }
}
void loop()
{
  // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        Run();
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}


