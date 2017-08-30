#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "AttitudeController.h"

enum AxisSelector{YAW_SELECTOR, PITCH_SELECTOR, ROLL_SELECTOR, AXIS_SELECTOR_MAX = ROLL_SELECTOR};

/**
 * Holds the tuning constant of the PID controller. 
 * Kp is the proportional gain
 * Ki is the integral gain
 * Kd is the derivative gain
 */
typedef struct PIDConstants{
  float Kp, Ki, Kd;
} PIDConstants;

/**
 * This class encapsulates all the PID controllers necessary to stabilize the quadcopter. More specifically,
 * there is one controller per axis (yaw / pitch / roll).
 */
class PIDController{

  private:    
    double pitchSetpoint, pitchInput, pitchControlOutput;
    double rollSetpoint, rollInput, rollControlOutput;
    PIDConstants pitchConstants, rollConstants;
    PIDConstants *currentConstantsSetting;
    
    PID pitchPID;
    PID rollPID;

    void refreshTunings(){
      pitchPID.SetTunings(pitchConstants.Kp, pitchConstants.Ki, pitchConstants.Kd);
      rollPID.SetTunings(rollConstants.Kp, rollConstants.Ki, rollConstants.Kd);
    }
  
  public:

    PIDController(): 
      pitchConstants{0.560, 0.100, 0.130}, 
      rollConstants{0.560, 0.100, 0.130},
      currentConstantsSetting{&pitchConstants},
      pitchSetpoint{0}, rollSetpoint{0},
      pitchInput{0}, rollInput{0},
      pitchPID{&pitchInput, &pitchControlOutput, &pitchSetpoint, pitchConstants.Kp, pitchConstants.Ki, pitchConstants.Kd, DIRECT},
      rollPID{&rollInput, &rollControlOutput, &rollSetpoint, rollConstants.Kp, rollConstants.Ki, rollConstants.Kd, DIRECT}{
    }

    void init(){
      /**
       * If the sampling interval is too high, the quadcopter will be unresponsive and this will lead to an unstable system.
       * If the sampling interval is too low, it will read twice the same output from the gyroscope and the derivative
       * part of the controller will jump up and down, again making it unstable.
       * 
       * The PID controller should be set to 100Hz.
       * 
       * Setting the PID mode on automatic turns the controller ON. On the contrary, setting it on MANUAL allows for manually
       * modifying the output variable.
       */
      pitchPID.SetSampleTime(10);
      pitchPID.SetOutputLimits(AttitudeController::MIN_PITCH_CONTROL, AttitudeController::MAX_PITCH_CONTROL);
      rollPID.SetSampleTime(10);
      rollPID.SetOutputLimits(AttitudeController::MIN_ROLL_CONTROL, AttitudeController::MAX_ROLL_CONTROL);

      pitchPID.SetMode(AUTOMATIC);
      rollPID.SetMode(AUTOMATIC);
    }

    void compute(){
      pitchPID.Compute();
      rollPID.Compute();
    }

    /**
     * This function should be called whenever the quadcopter is not in a situation where it can 
     * stabilize itself. For example, if the motors are not spinning or during an emergency break
     */
    void disable(){
      if (pitchPID.GetMode() == MANUAL && rollPID.GetMode() == MANUAL){
        return;
      } else {
        pitchPID.SetMode(MANUAL);
        rollPID.SetMode(MANUAL);

        pitchControlOutput = rollControlOutput = 0;
      }
    }

    void enable(){
      if (pitchPID.GetMode() == AUTOMATIC && rollPID.GetMode() == AUTOMATIC){
        return;
      } else {
        pitchPID.SetMode(AUTOMATIC);
        rollPID.SetMode(AUTOMATIC);
      }
    }

    /**
     * Because there is a PID controller for each axis, and each of them has 3 gain constants,
     * a pointer to a PIDConstants struct is kept. This function allows to swap between the 
     * structs
     */
    void swapPIDSettingAxis(){
      if(currentConstantsSetting == &pitchConstants){
        currentConstantsSetting = &rollConstants;
      } else{
        currentConstantsSetting = &pitchConstants;
      }
    }

    
    void updateSetpoints(float pS, float rS){
      this->pitchSetpoint = pS;
      this->rollSetpoint = rS;
    }

    void updateInputs(float pI, float rI){
      this->pitchInput = pI;
      this->rollInput = rI;
    }

    /**
     * Updates the proportional gain of the currently pointed PIDConstant
     */
    void updateKp(float delta){
      currentConstantsSetting->Kp += delta;
      refreshTunings();
    }

    /**
     * Updates the integral gain of the currently pointed PIDConstant
     */
    void updateKi(float delta){
      currentConstantsSetting->Ki += delta;
      refreshTunings();
    }

    /**
     * Update the derivative gain of the currently pointed PIDConstant
     */
    void updateKd(float delta){
      currentConstantsSetting->Kd += delta;
      refreshTunings();
    }

    float getKp(){
      return currentConstantsSetting->Kp;
    }

    float getKi(){
      return currentConstantsSetting->Ki;
    }

    float getKd(){
      return currentConstantsSetting->Kd;
    }
    
    float getPitchOutput(){
      return pitchControlOutput;
    }
    
    float getRollOutput(){
      return rollControlOutput;
    }

};

#endif
