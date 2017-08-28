#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include "AttitudeController.h"

enum AxisSelector{YAW_SELECTOR, PITCH_SELECTOR, ROLL_SELECTOR, AXIS_SELECTOR_MAX = ROLL_SELECTOR};

typedef struct PIDConstants{
  float Kp, Ki, Kd;
} PIDConstants;

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
      rollPID{&rollInput, &rollControlOutput, &rollSetpoint, rollConstants.Kp, rollConstants.Ki, rollConstants.Kd, REVERSE}{
    }

    void init(){
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

    void updateKp(float delta){
      currentConstantsSetting->Kp += delta;
      refreshTunings();
    }

    void updateKi(float delta){
      currentConstantsSetting->Ki += delta;
      refreshTunings();
    }

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
