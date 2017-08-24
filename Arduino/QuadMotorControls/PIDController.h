#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

class PIDController{

  private:
    const int DEFAULT_CONTROL_BOUND = 50;
    const int MIN_YAW_CONTROL = -DEFAULT_CONTROL_BOUND;
    const int MAX_YAW_CONTROL = DEFAULT_CONTROL_BOUND;
    const int MIN_PITCH_CONTROL = -DEFAULT_CONTROL_BOUND;
    const int MAX_PITCH_CONTROL = DEFAULT_CONTROL_BOUND;
    const int MIN_ROLL_CONTROL = -DEFAULT_CONTROL_BOUND;
    const int MAX_ROLL_CONTROL = DEFAULT_CONTROL_BOUND;
    const float PIDConstantStep = 0.01;
    
    double pitchSetpoint, pitchInput, pitchControlOutput;
    double rollSetpoint, rollInput, rollControlOutput;
    double Kp, Ki, Kd;
    PID pitchPID;
    PID rollPID;
  
  public:

    PIDController(): Kp{0.50}, Ki{0.15}, Kd{0.05}, 
      pitchPID{&pitchInput, &pitchControlOutput, &pitchSetpoint, Kp, Ki, Kd, DIRECT},
      rollPID{&rollInput, &rollControlOutput, &rollSetpoint, Kp, Ki, Kd, REVERSE}{
      
    }

    void init(){
      //pitchPID.SetSampleTime(50);
      pitchPID.SetOutputLimits(MIN_PITCH_CONTROL, MAX_PITCH_CONTROL);
      //rollPID.SetSampleTime(50);
      rollPID.SetOutputLimits(MIN_ROLL_CONTROL, MAX_ROLL_CONTROL);
    }
};

#endif
