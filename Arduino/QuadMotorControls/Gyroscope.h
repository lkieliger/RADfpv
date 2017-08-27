#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
   mpuInterrupt = true;
}

class Gyroscope{
  private:
    float yaw, pitch, roll;
    const float TO_DEG = 180.0/M_PI;
    MPU6050 mpu;
    
    // MPU control/status vars
    bool dmpReady = false;  // set true if DMP init was successful
    uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
    uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer

    // orientation/motion vars
    Quaternion q;           // [w, x, y, z]         quaternion container
    VectorInt16 aa;         // [x, y, z]            accel sensor measurements
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
    VectorFloat gravity;    // [x, y, z]            gravity vector
    float euler[3];         // [psi, theta, phi]    Euler angle container
    
    //TODO: simplify with variables yaw / pitch / roll
    float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

  public:
    Gyroscope(): yaw{0}, pitch{0}, roll{0}, mpu{}, ypr{0,0,0}{
      
    }

    void init(){
      // join I2C bus (I2Cdev library doesn't do this automatically)
      #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
          Wire.begin();
          Wire.setClock(400000); 
      #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
          Fastwire::setup(400, true);
      #endif
      Serial.begin(230400);
      while (!Serial);
  
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
  
      mpu.setXGyroOffset(-107);
      mpu.setYGyroOffset(-4);
      mpu.setZGyroOffset(4);
      mpu.setXAccelOffset(-3011);
      mpu.setYAccelOffset(-2583);
      mpu.setZAccelOffset(1061); 
  
      //default is 4 -> 100Hz
      mpu.setRate(3); 
  
      // make sure it worked (returns 0 if so)
      if (devStatus == 0) {
          // turn on the DMP, now that it's ready
          Serial.println(F("Enabling DMP..."));
          mpu.setDMPEnabled(true);

          Serial.print(F("DLPF Mode: "));
          Serial.println(mpu.getDLPFMode());
          Serial.print(F("Gyro outputrate divider: "));
          Serial.println(mpu.getRate());
          
          // enable Arduino interrupt detection
          Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
          attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
          mpuIntStatus = mpu.getIntStatus();
  
          // set DMP Ready flag so the main loop() function knows it's okay to use it
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
    }

    boolean isReady(){
      return mpuInterrupt || fifoCount >= packetSize;
    }
    
    void actualize(){
      // if programming failed, don't try to do anything
      if (!dmpReady) return;
      
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
  
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

          yaw = ypr[0] * TO_DEG;
          pitch = ypr[1] * TO_DEG;
          roll = ypr[2] * TO_DEG;
      }
    }

    float getYaw(){
      return yaw;
    }

    float getPitch(){
      return pitch;
    }

    float getRoll(){
      return roll;
    }
  
};

#endif
