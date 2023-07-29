#include <Arduino.h>
#include <AccelStepper.h>

// BEGIN STEPPER DEFINES
#define FLAPSGAUGE_STEPPER_PIN_1 (0)
#define FLAPSGAUGE_STEPPER_PIN_2 (1)
#define FLAPSGAUGE_STEPPER_PIN_3 (2)
#define FLAPSGAUGE_STEPPER_PIN_4 (3)

#define STEPPER_MAX_SPEED (100)
#define STEPPER_HOMING_SPEED (100)
#define STEPPER_ACCELERATION (50)

#define FLAPSGAUGE_STEPS_PER_REVOLUTION (2048)
#define FLAPSGAUGE_ANGLE (270) // angle from 0 to full on flaps gauge

// uncomment as desired
#define FLAPSGAUGE_MANUAL_HOME_PIN (5)
#define FLAPSGAUGE_MANUAL_HOME_LEVEL (LOW)
#define FLAPSGAUGE_MANUAL_HOME_WAIT_TIME (1000)
// #define FLAPSGAUGE_AUTO_HOME_PIN (4) 
// #define FLAPSGAUGE_AUTO_HOME_LEVEL (LOW)

// END STEPPER DEFINES


// BEGIN STEPPER VARIABLES
// Common type definitions for all gauges
typedef enum {
  STEPPER_HOMING_AUTO,
  STEPPER_HOMING_MANUAL,
  STEPPER_HOMING_MANUAL_WAIT,
  STEPPER_OPERATING
} StepperMode;

// replicate the following lines for brake gauge
AccelStepper flapsGauge(
  AccelStepper::MotorInterfaceType::FULL4WIRE, 
  FLAPSGAUGE_STEPPER_PIN_1, 
  FLAPSGAUGE_STEPPER_PIN_2, 
  FLAPSGAUGE_STEPPER_PIN_3, 
  FLAPSGAUGE_STEPPER_PIN_4);
StepperMode flapsGaugeStepperMode;
FlightSimFloat flapsGaugePosition;
elapsedMillis flapsGaugeStepperHomeManualWait;
// END STEPPER VARIABLES

void setup() {
  Serial.begin(9600);

  // BEGIN STEPPER SETUP CODE
  flapsGaugePosition=XPlaneRef("sim/cockpit2/controls/flap_handle_deploy_ratio");

  flapsGauge.setMaxSpeed(STEPPER_MAX_SPEED);
  flapsGauge.setAcceleration(STEPPER_ACCELERATION);

#ifdef FLAPSGAUGE_AUTO_HOME_PIN
  flapsGaugeStepperMode = STEPPER_HOMING_AUTO;
  pinMode(FLAPSGAUGE_AUTO_HOME_PIN, INPUT_PULLUP);
#else
  flapsGaugeStepperMode = STEPPER_OPERATING;
#endif

#ifdef FLAPSGAUGE_MANUAL_HOME_PIN
  pinMode(FLAPSGAUGE_MANUAL_HOME_PIN, INPUT_PULLUP);
#endif

  // END STEPPER SETUP CODE
}

void loop() {
  FlightSim.update();

  // BEGIN STEPPER LOOP CODE
  if (flapsGaugeStepperMode == STEPPER_OPERATING) {
#ifdef FLAPSGAUGE_MANUAL_HOME_PIN
    if (digitalReadFast(FLAPSGAUGE_MANUAL_HOME_PIN) == FLAPSGAUGE_MANUAL_HOME_LEVEL) {
      flapsGaugeStepperMode = STEPPER_HOMING_MANUAL;
    }
#endif
    float flapsAngle = flapsGaugePosition * FLAPSGAUGE_ANGLE;
    int stepperPosition = flapsAngle / 360.0 * FLAPSGAUGE_STEPS_PER_REVOLUTION;
    flapsGauge.moveTo(stepperPosition); 
    flapsGauge.run();
  } else if (flapsGaugeStepperMode == STEPPER_HOMING_AUTO) {
    // turn stepper
    flapsGauge.setSpeed(STEPPER_HOMING_SPEED);
    flapsGauge.runSpeed();
#ifdef FLAPSGAUGE_AUTO_HOME_PIN
    if (digitalReadFast(FLAPSGAUGE_AUTO_HOME_PIN) == FLAPSGAUGE_AUTO_HOME_LEVEL) {
      // home position reached, switch to normal operation mode
      flapsGauge.setCurrentPosition(0);
      stepperMode = STEPPER_OPERATING;
    }
#endif
  } else if (flapsGaugeStepperMode == STEPPER_HOMING_MANUAL)  {
    // turn stepper
    flapsGauge.setSpeed(STEPPER_HOMING_SPEED);
    flapsGauge.runSpeed();
#ifdef FLAPSGAUGE_MANUAL_HOME_PIN
    if (digitalReadFast(FLAPSGAUGE_MANUAL_HOME_PIN) != FLAPSGAUGE_MANUAL_HOME_LEVEL) {
      // go into waiting mode before switching to operational mode
      flapsGaugeStepperMode = STEPPER_HOMING_MANUAL_WAIT;
      flapsGaugeStepperHomeManualWait = 0;
    }
#endif
  } 
  else if (flapsGaugeStepperMode == STEPPER_HOMING_MANUAL_WAIT)  {
    if (flapsGaugeStepperHomeManualWait >= FLAPSGAUGE_MANUAL_HOME_WAIT_TIME) {
      // timeout expired, go to normal operation mode
      flapsGaugeStepperMode = STEPPER_OPERATING;
      flapsGauge.setCurrentPosition(0);
    }
#ifdef FLAPSGAUGE_MANUAL_HOME_PIN
    if (digitalReadFast(FLAPSGAUGE_MANUAL_HOME_PIN) == FLAPSGAUGE_MANUAL_HOME_LEVEL) {
      // back to manual homing mode
      flapsGaugeStepperMode = STEPPER_HOMING_MANUAL;
    }
#endif
  } 
  // END STEPPER LOOP CODE
}
