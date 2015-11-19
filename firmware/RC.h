#ifndef RC_H
#define RC_H

#include <Arduino.h>
/**
 * RC_Controller: Class used to read and handle commands sent by an attached RC controller
 *                The class reads auxiliary, throttle and rudder channels and converts from
 *                PPM into boolean, float and float respectively. These values can be read
 *                using the appropriate accessor methods. The class assumes the following
 *                functionality:
 *                AUX Pin is low:  Ignore throttle and rudder input
 *                AUX Pin is high: Read throttle and rudder input. Throttle and rudder
 *                                 output have no effect until the controller is armed using
 *                                 the command: throttle down, rudder left. After 5 seconds
 *                                 the same sequence will disarm the controller.
 *                                 A throttle down and rudder right command will force the
 *                                 esc into calibration mode. This process will block for
 *                                 10 seconds.
 *
 *                                 Once armed the throttle and rudder commands will be mixed
 *                                 to give a velocity for each motor
 */
class RC_Controller{
  public:
    RC_Controller();
    RC_Controller(int aux_pin, int throttle_pin, int rudder_pin );

    void setLeftRudder(int lr);
    void setRightRudder(int rr);
    void setMaxThrottle(int mt);
    void setMinThrottle(int mt);
    void setAuxLow(int al);
    void setAuxHigh(int ah);
    void setMotorUpdateBlocked(bool state);

    bool  isCalibrateEnabled();
    bool  isOverrideEnabled();
    bool  isArmed();
    bool isMotorUpdateBlocked();
    
    float throttleVal();
    float rudderVal();
    float rightVelocity();
    float leftVelocity();

    void update();

  private:
      //RC Controller Ouptut Values
    int min_throttle = 1169;
    int max_throttle = 1990;

    int left_rudder = 1162;
    int right_rudder = 1975;

    int aux_low = 900;
    int aux_high = 2000;
    int aux_threshold = 1300;

    bool  calibrate = false;
    bool  overrideEnabled = false;
    bool  armed = false;
    bool  motorUpdateBlocked = false;
    float throttle_val = 0;
    float rudder_val = 0;
    int   aux_val;

};


#endif
