#include "RC.h"

//Rx input pins
int THROTTLE_PIN = 3;
int RUDDER_PIN = 2;
int AUX_PIN = 20;

//Throttle and Rudder Values as sent by RC
volatile uint32_t throttle_pwm = 0;
volatile uint32_t rudder_pwm = 0;
volatile uint32_t aux_pwm = 0;

volatile bool RCoverride = false;

//auxiliary pin listener
void auxInterrupt(  )
{
  static uint32_t auxStartTime;
  //Start timer on rising edge
  if(digitalRead(AUX_PIN))
  {
    auxStartTime = micros();
  }
  //Stop timer on falling edge
  else
  {
    //Compute pulse duration
    aux_pwm = (uint32_t)(micros() - auxStartTime);

  }
}

//Throttle pin listener
void throttleInterrupt(  )
{
  static uint32_t throttleStartTime;

  if(digitalRead(THROTTLE_PIN))
  {
    throttleStartTime = micros();
  }
  else
  {
    throttle_pwm = (uint32_t)(micros() - throttleStartTime);
  }

}

//Rudder pin listener
void rudderInterrupt(  )
{
  static uint32_t rudderStartTime;

  if(digitalRead(RUDDER_PIN))
  {
    rudderStartTime = micros();
  }
  else
  {
    rudder_pwm = (uint32_t)(micros() - rudderStartTime);
  }

}

/**
 * Rc_Controller default constructor
 * Enable aux, throttle and rudder pins to be used for interrupts
 * Attach interrupt to auxiliary pin to listen for manual override
 *
 * Default pins are used as defined in header file unless custom
 * constructor was called
 */
RC_Controller::RC_Controller()
{
  pinMode(THROTTLE_PIN, INPUT); digitalWrite(THROTTLE_PIN, HIGH);
  pinMode(RUDDER_PIN, INPUT);   digitalWrite(RUDDER_PIN, HIGH);
  pinMode(AUX_PIN, INPUT);   digitalWrite(AUX_PIN, HIGH);

  attachInterrupt(AUX_PIN, auxInterrupt, CHANGE);

}

/**
 * Custom Constructor to attach Rx module to given pins
 * @param aux_pin: Input pin for auxiliary channel
 * @param throttle_pin: Input pin for throttle channel
 * @param rudder_pin: Input pin for rudder channel
 */
RC_Controller::RC_Controller(int aux_pin, int throttle_pin, int rudder_pin)
{
  //Assign pins
  AUX_PIN = aux_pin;
  THROTTLE_PIN = throttle_pin;
  RUDDER_PIN = rudder_pin;

  //Call default constructor to initialise pins
  RC_Controller();
}

//Mutator methods used to set channel input limits
//Used for calibration routine.
void RC_Controller::setLeftRudder(int lr)  { left_rudder = lr; }
void RC_Controller::setRightRudder(int rr) { right_rudder = rr; }
void RC_Controller::setMaxThrottle(int mt) { max_throttle = mt; }
void RC_Controller::setMinThrottle(int mt) { min_throttle = mt; }

void RC_Controller::setAuxLow(int al)
{
  aux_low = al;
  //Update threshold for auxiliary pin decision
  aux_threshold = (aux_low + aux_high)/2;
}

void RC_Controller::setAuxHigh(int ah)
{
  aux_high = ah;
  //Update threshold for auxiliary pin decision
  aux_threshold = (aux_low + aux_high)/2;
}

//Accessor Methods
bool  RC_Controller::isOverrideEnabled() { return overrideEnabled; }
bool  RC_Controller::isArmed() { return armed; }
float RC_Controller::throttleVal() { return throttle_val; }
float RC_Controller::rudderVal() { return rudder_val; }
bool  RC_Controller::isCalibrateEnabled() {
  if (calibrate)
  {
    calibrate = false;
    return true;   
  }
  else
    return false;
}

bool RC_Controller::isMotorUpdateBlocked() { return motorUpdateBlocked; }

void RC_Controller::setMotorUpdateBlocked(bool state) { motorUpdateBlocked = state; }

//Velocity mixers
//Implemented as simple linear mixers. Turn speed is controlled by
//the position of the throttle stick. Turning angle is linearly proportional
//to the position of the rudder

//Returns the velocity for the right motor
float RC_Controller::rightVelocity()
{
  //Turning left -> reduce right motor speed
  if (rudder_val < 0)
  {
    return (1+rudder_val)*throttle_val;
  }
  else
  {
    return throttle_val;
  }
}

//Returns the velocity for the right motor
float RC_Controller::leftVelocity()
{
  //Turning right -> reduce left motor speed
  if (rudder_val > 0)
  {
    return (1-rudder_val)*throttle_val;
  }
  else
  {
    return throttle_val;
  }
}

//RC Update loop
//Reads channel inputs if available
//Sets Override flag and arming + calibration routine
void RC_Controller::update()
{
      //Time since last arming state change
      //Limit to 5 seconds between arm and disarm
      static long armed_time = 0;

      //Turn off interrupts to update local variables
      noInterrupts();

      //Update local variables
      aux_val = aux_pwm;
      //AUX value is below threshold or outside of valid window
      //set override to false
      if(aux_val < aux_threshold || aux_val > aux_high)
      {
        //If override was previously enabled, then
        //stop listening to throttle and rudder pins
        if(overrideEnabled)
        {
          detachInterrupt(THROTTLE_PIN);
          detachInterrupt(RUDDER_PIN);
          throttle_val = 0;
          rudder_val = 0;

        }

        overrideEnabled = false;
        armed = false;

      }
      //AUX pin is above threshold
      else
      {
        //Attach throttle and rudder listeners
        //if override was previously off
        if(!overrideEnabled)
        {
         attachInterrupt(RUDDER_PIN, rudderInterrupt, CHANGE);
         attachInterrupt(THROTTLE_PIN, throttleInterrupt, CHANGE);
        }
        overrideEnabled = true;
        //Zero throttle and rudder values to prevent false readings
        throttle_val = 0;
        rudder_val = 0;

      }

      //Read throttle and rudder values if auxiliary pin was high
      if(overrideEnabled )
      {
        throttle_val = (float)map(throttle_pwm, min_throttle, max_throttle, 0, 100)/100.0;
        rudder_val   = (float)map(rudder_pwm,left_rudder, right_rudder, -100, 100)/100.0;
      }


      //Local update complete - turn on interrupts
      interrupts();


      //RC commands are being received - deal with calibration and arming
      if(overrideEnabled)
      {
          //Throttle down and rudder left: Change ESC arm state
          if(throttle_val < 0.05 && rudder_val < -0.95 && ( millis()-armed_time > 5000))
          {
              armed = !armed;
              armed_time = millis();

          }
          //Throttle down and rudder right: calibrate ESCs
          else if(throttle_val < 0.05 &&  rudder_val > 0.95)
          {
            Serial.println("In calibrate");
            calibrate = true;
          }
          else if(!armed)
          {
            throttle_val = 0;
            rudder_val = 0;
          }

       }

}
