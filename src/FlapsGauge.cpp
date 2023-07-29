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
typedef enum {
  STEPPER_HOMING_AUTO,
  STEPPER_HOMING_MANUAL,
  STEPPER_HOMING_MANUAL_WAIT,
  STEPPER_OPERATING
} StepperMode;

AccelStepper flapsGauge(
  AccelStepper::MotorInterfaceType::FULL4WIRE, 
  FLAPSGAUGE_STEPPER_PIN_1, 
  FLAPSGAUGE_STEPPER_PIN_2, 
  FLAPSGAUGE_STEPPER_PIN_3, 
  FLAPSGAUGE_STEPPER_PIN_4);

StepperMode stepperMode;
FlightSimFloat flapsGaugePosition;
elapsedMillis stepperHomeManualWait;
// END STEPPER VARIABLES

void setup() {
  Serial.begin(9600);

  // BEGIN STEPPER SETUP CODE
  flapsGaugePosition=XPlaneRef("sim/cockpit2/controls/flap_handle_deploy_ratio");

  flapsGauge.setMaxSpeed(STEPPER_MAX_SPEED);
  flapsGauge.setAcceleration(STEPPER_ACCELERATION);

#ifdef FLAPSGAUGE_AUTO_HOME_PIN
  stepperMode = STEPPER_HOMING;
  pinMode(FLAPSGAUGE_AUTO_HOME_PIN, INPUT_PULLUP);
#else
  stepperMode = STEPPER_OPERATING;
#endif

#ifdef FLAPSGAUGE_MANUAL_HOME_PIN
  pinMode(FLAPSGAUGE_MANUAL_HOME_PIN, INPUT_PULLUP);
#endif

  // END STEPPER SETUP CODE
}

void loop() {
  FlightSim.update();

  // Stepper code
  if (stepperMode == STEPPER_HOMING_AUTO) {
    flapsGauge.setSpeed(STEPPER_HOMING_SPEED);
    flapsGauge.runSpeed();
#ifdef FLAPSGAUGE_AUTO_HOME_PIN
    if (digitalReadFast(FLAPSGAUGE_AUTO_HOME_PIN) == FLAPSGAUGE_AUTO_HOME_LEVEL) {
      flapsGauge.setCurrentPosition(0);
      stepperMode = STEPPER_OPERATING;
    }
#endif
  } else if (stepperMode == STEPPER_HOMING_MANUAL)  {
    flapsGauge.setSpeed(STEPPER_HOMING_SPEED);
    flapsGauge.runSpeed();
#ifdef FLAPSGAUGE_MANUAL_HOME_PIN
    if (digitalReadFast(FLAPSGAUGE_MANUAL_HOME_PIN) != FLAPSGAUGE_MANUAL_HOME_LEVEL) {
      flapsGauge.setCurrentPosition(0);
      stepperMode = STEPPER_HOMING_MANUAL_WAIT;
      stepperHomeManualWait = 0;
    }
#endif
  } 
  else if (stepperMode == STEPPER_HOMING_MANUAL)  {
    if (stepperHomeManualWait >= FLAPSGAUGE_MANUAL_HOME_WAIT_TIME) {
      stepperMode = STEPPER_OPERATING;
    }
#ifdef FLAPSGAUGE_MANUAL_HOME_PIN
    if (digitalReadFast(FLAPSGAUGE_MANUAL_HOME_PIN) != FLAPSGAUGE_MANUAL_HOME_LEVEL) {
      flapsGauge.setCurrentPosition(0);
      stepperMode = STEPPER_HOMING_MANUAL_WAIT;
      stepperHomeManualWait = 0;
    }
#endif
  } 
  else if (stepperMode == STEPPER_OPERATING) {
#ifdef FLAPSGAUGE_MANUAL_HOME_PIN
    if (digitalReadFast(FLAPSGAUGE_MANUAL_HOME_PIN) == FLAPSGAUGE_MANUAL_HOME_LEVEL) {
      stepperMode = STEPPER_HOMING_MANUAL;
    }
#endif
    float flapsAngle = flapsGaugePosition * FLAPSGAUGE_ANGLE;
    int stepperPosition = flapsAngle / 360.0 * FLAPSGAUGE_STEPS_PER_REVOLUTION;
    flapsGauge.moveTo(stepperPosition); 
    flapsGauge.run();
  }

}
