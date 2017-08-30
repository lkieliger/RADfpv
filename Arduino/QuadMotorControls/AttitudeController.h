#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H

/**
 * This class takes the navigation commands on all three axis (yaw / pitch / roll) and translate them
 * into motor thrust so that the quadcopter can reach the desired attitude.
 */
class AttitudeController {

  public:
    static const short DEFAULT_CONTROL_BOUND = 50;
    static const short MIN_YAW_CONTROL = -DEFAULT_CONTROL_BOUND;
    static const short MAX_YAW_CONTROL = DEFAULT_CONTROL_BOUND;
    static const short MIN_PITCH_CONTROL = -DEFAULT_CONTROL_BOUND;
    static const short MAX_PITCH_CONTROL = DEFAULT_CONTROL_BOUND;
    static const short MIN_ROLL_CONTROL = -DEFAULT_CONTROL_BOUND;
    static const short MAX_ROLL_CONTROL = DEFAULT_CONTROL_BOUND;

    static constexpr short STARTUP_THRUST = 1000;

    static constexpr short NUM_MOTORS = 4;

    static constexpr short FRONT_LEFT = 0;
    static constexpr short FRONT_RIGHT = 1;
    static constexpr short BACK_RIGHT = 2;
    static constexpr short BACK_LEFT = 3;
    
    AttitudeController(): motors{{5}, {6}, {10}, {11}}, baseThrust{0}, yawControl{0}, pitchControl{0}, rollControl{0} {

    }

    /**
     * A low signal should initially be sent to the ESC to arm them. If the ESC read a high signal, they will enter
     * calibration mode.
     */
    void init() {
      for (int i = 0; i < NUM_MOTORS; i++)
      {
        motors[i].init();
        motors[i].writeThrust();
      }
    }

    /**
     * Instantly stops the motors. 0 is not a valid value to stop the motors because they will continue spinning
     * due to their inertia. This function actively blocks the motors to avoid physical damage to the quadcopter
     * or the person manoeuvering it.
     */
    void emergencyBrake(){
      baseThrust = 0;
      for (int i = 0; i < NUM_MOTORS; i++)
      {
        motors[i].setThrust(BRAKE_THRUST);
        motors[i].writeThrust();
      }
    }

    /**
     * The base thrust is the amount of thust that each motor initially has. To change the quadcopter's attitude,
     * the thrust of each motor is then adjusted with respect to the base thrust.
     */
    void increaseBaseThrust() {
      baseThrust = constrain(baseThrust + THRUST_STEP, MIN_THRUST, MAX_THRUST);
    }

    void decreaseBaseThrust() {
      baseThrust = constrain(baseThrust - THRUST_STEP, MIN_THRUST, MAX_THRUST);
    }

    void shutdownMotors() {
      baseThrust = MIN_THRUST;
    }

    void startupMotors() {
      baseThrust = STARTUP_THRUST;
    }

    void rotateYaw(int d) {
      yawControl = constrain(d, MIN_YAW_CONTROL, MAX_YAW_CONTROL);
    }

    void rotatePitch(int d) {
      pitchControl = constrain(d, MIN_PITCH_CONTROL, MAX_PITCH_CONTROL);
    }

    void rotateRoll(int d) {
      rollControl = constrain(d, MIN_ROLL_CONTROL, MAX_ROLL_CONTROL);
    }

    void resetControls() {
      yawControl = pitchControl = rollControl = 0;
    }

    void outputThrustSignal() {
      /**
         Yaw is the vertical axis
         Pitch is the lateral axis
         Roll is the longitudinal axis
      */
      for (int i = 0; i < NUM_MOTORS; i++)
      {
        motors[i].setThrust(baseThrust);
      }

      if (yawControl != 0) {
        /**
           Unbalance angular momentum of the propellers, the body of the drone pivots in the opposite direction to
           keep the total angular momentum unchanged. Increase thrust on the other motors to keep the total forward force
           unchanged.
        */
        motors[FRONT_RIGHT].updateThrust( - yawControl);
        motors[BACK_LEFT].updateThrust( - yawControl);

        motors[FRONT_LEFT].updateThrust( + yawControl);
        motors[BACK_RIGHT].updateThrust( + yawControl);
      }

      if (pitchControl != 0) {
        motors[FRONT_LEFT].updateThrust( + pitchControl);
        motors[FRONT_RIGHT].updateThrust( + pitchControl);

        motors[BACK_RIGHT].updateThrust( - pitchControl);
        motors[BACK_LEFT].updateThrust( - pitchControl);
      }

      if (rollControl != 0) {
        motors[FRONT_LEFT].updateThrust( + rollControl);
        motors[BACK_LEFT].updateThrust( + rollControl);

        motors[FRONT_RIGHT].updateThrust( - rollControl);
        motors[BACK_RIGHT].updateThrust( - rollControl);
      }

      for (int i = 0; i < NUM_MOTORS; i++)
      {
        motors[i].writeThrust();
      }
    }

    short getYawControl() {
      return yawControl;
    }

    short getPitchControl() {
      return pitchControl;
    }

    short getRollControl() {
      return rollControl;
    }

    short getBaseThrust(){
      return baseThrust;
    }

    short getMotorThrust(short i){
      if (i > 3 || i < 0){
        return -1;
      }
      
      return motors[i].getThrust();
    }

  private:

    // Motors can actually go up to ~2000. This is a safeguard.
    static constexpr short MAX_THRUST = 1400;
    static constexpr short MIN_THRUST = 0;
    static constexpr short BRAKE_THRUST = STARTUP_THRUST;
   
    static constexpr short THRUST_STEP = 10;
    
    short baseThrust;
    short yawControl, pitchControl, rollControl;

    Motor motors[4];
};
#endif
