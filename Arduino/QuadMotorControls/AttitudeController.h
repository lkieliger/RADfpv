#ifndef ATTITUDE_CONTROLLER_H
#define ATTITUDE_CONTROLLER_H


class AttitudeController {

  public:
    AttitudeController(): motors{{5}, {6}, {10}, {11}}, baseThrust{0}, yawControl{0}, pitchControl{0}, rollControl{0} {

    }

    void init() {
      // Send a low signal initially for normal mode
      for (int i = 0; i < NUM_MOTORS; i++)
      {
        motors[i].init();
        motors[i].writeThrust();
      }
    }

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

    int getYawControl() {
      return yawControl;
    }

    int getPitchControl() {
      return pitchControl;
    }

    int getRollControl() {
      return rollControl;
    }

  private:

    // Motors can actually go up to ~2000. This is a safeguard.
    static constexpr int MAX_THRUST = 1500;
    static constexpr int MIN_THRUST = 0;
    static constexpr int STARTUP_THRUST = 1000;

    static constexpr int THRUST_STEP = 10;

    static constexpr int NUM_MOTORS = 4;

    static constexpr int FRONT_LEFT = 0;
    static constexpr int FRONT_RIGHT = 1;
    static constexpr int BACK_RIGHT = 2;
    static constexpr int BACK_LEFT = 3;

    const int DEFAULT_CONTROL_BOUND = 50;
    const int MIN_YAW_CONTROL = -DEFAULT_CONTROL_BOUND;
    const int MAX_YAW_CONTROL = DEFAULT_CONTROL_BOUND;
    const int MIN_PITCH_CONTROL = -DEFAULT_CONTROL_BOUND;
    const int MAX_PITCH_CONTROL = DEFAULT_CONTROL_BOUND;
    const int MIN_ROLL_CONTROL = -DEFAULT_CONTROL_BOUND;
    const int MAX_ROLL_CONTROL = DEFAULT_CONTROL_BOUND;

    int baseThrust;
    int yawControl, pitchControl, rollControl;

    Motor motors[4];
};
#endif
