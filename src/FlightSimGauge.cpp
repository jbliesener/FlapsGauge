#include "FlightSimGauge.h"

FlightSimGauge::FlightSimGauge(const char* gaugeName, AccelStepper &gaugeStepper) :
    name(gaugeName),
    stepper(gaugeStepper)
 {
    mode=GAUGE_OPERATING;
    timer = 0;
    debugMode = false;
    manualHomingPin = -1;
    autoHomingPin = -1;
    isLimitHoming = false;
    homingLevel = LOW;
    manualHomingPressDelay = 100;
    manualHomingReleaseDelay = 1000;
    homingPosition = 0;
}

void FlightSimGauge::setManualHomingPinAndSpeed(int16_t pinNumber, uint8_t activeLevel, int32_t homingSpeed) {
    manualHomingPin = pinNumber;
    homingLevel = activeLevel;
    this->homingSpeed = homingSpeed;
    autoHomingPin = -1;
    isLimitHoming = false;
    mode=GAUGE_OPERATING;
    if (activeLevel == LOW) {
        pinMode(pinNumber, INPUT_PULLUP);
    } else if (activeLevel == HIGH) {
        pinMode(pinNumber, INPUT_PULLDOWN);
    }
}

void FlightSimGauge::setManualHomingPinTiming(uint32_t pressDelay, uint32_t releaseDelay) {
    manualHomingPressDelay = pressDelay;
    manualHomingReleaseDelay = releaseDelay;
}

void FlightSimGauge::setAutoHomingPinAndSpeed(int16_t pinNumber, uint8_t activeLevel, int32_t homingSpeed) {
    autoHomingPin = pinNumber;
    homingLevel = activeLevel;
    manualHomingPin = -1;
    isLimitHoming = false;
    mode=GAUGE_HOMING_AUTO;
    stepper.setSpeed(homingSpeed);
    if (activeLevel == LOW) {
        pinMode(pinNumber, INPUT_PULLUP);
    } else if (activeLevel == HIGH) {
        pinMode(pinNumber, INPUT_PULLDOWN);
    }
}

void FlightSimGauge::setLimitHoming(int32_t steps) {
    manualHomingPin = -1;
    autoHomingPin = -1;
    isLimitHoming = true;
    mode = GAUGE_HOMING_LIMIT;

    limitHomingPosition = steps;   
    stepper.moveTo(limitHomingPosition);
}

void FlightSimGauge::loop() {
    switch (mode) {
        case GAUGE_OPERATING: 
            stepper.run();
            if (manualHomingPin != -1) {
                if (digitalReadFast(manualHomingPin) == homingLevel) {
                    setMode(GAUGE_HOMING_WAIT_PRESS);
                    timer = 0;
                }
            }
            break;

        case GAUGE_HOMING_WAIT_PRESS:
            if (digitalReadFast(manualHomingPin) != homingLevel) {
                setMode(GAUGE_OPERATING);
                break;
            }
            if (timer > manualHomingPressDelay) {
                setMode(GAUGE_HOMING_MANUAL);
                stepper.setSpeed(homingSpeed);
            }
            break;

        case GAUGE_HOMING_MANUAL:
            if (digitalReadFast(manualHomingPin) != homingLevel) {
                timer = 0;
                setMode(GAUGE_HOMING_WAIT_RELEASE);
                break;
            }
            stepper.runSpeed();
            break;

        case GAUGE_HOMING_WAIT_RELEASE:
            if (digitalReadFast(manualHomingPin) == homingLevel) {
                setMode(GAUGE_HOMING_MANUAL);
            }
            if (timer > manualHomingReleaseDelay) {
                stepper.setCurrentPosition(homingPosition);
                setMode(GAUGE_OPERATING);
            }
            break;

        case GAUGE_HOMING_AUTO:
            if (digitalReadFast(autoHomingPin) == homingLevel) {
                stepper.setCurrentPosition(homingPosition);
                setMode(GAUGE_OPERATING);
                break;
            }
            stepper.runSpeed();
            break;

        case GAUGE_HOMING_LIMIT:
            if (stepper.currentPosition() == limitHomingPosition) {
                stepper.setCurrentPosition(homingPosition);
                setMode(GAUGE_OPERATING);
                break;
            }
            stepper.runSpeed();
            break;
    }
    if (debugMode && debugTimer > debugInterval) {
        debugTimer = 0;
        Serial.printf("%10lu: Gauge %s, mode %s, current position: %d, target position %d\n", millis(), 
            name, getModeString(mode),
            stepper.currentPosition(), stepper.targetPosition());
    }
 }

const char *FlightSimGauge::getModeString(FlightSimGaugeMode mode) {
    switch (mode) {
        case GAUGE_OPERATING:           return "OPERATING";           break;
        case GAUGE_HOMING_WAIT_PRESS:   return "HOMING_WAIT_PRESS";   break;
        case GAUGE_HOMING_MANUAL:       return "HOMING_MANUAL";       break;
        case GAUGE_HOMING_WAIT_RELEASE: return "HOMING_WAIT_RELEASE"; break;
        case GAUGE_HOMING_AUTO:         return "HOMING_AUTO";         break;
        case GAUGE_HOMING_LIMIT:        return "HOMING_LIMIT";        break;
        default: return "UNKNOWN"; break;
    }
}

void FlightSimGauge::setMode(FlightSimGaugeMode mode) {
    if (debugMode) {
        Serial.printf("%10lu: Switching gauge %s from mode %s to mode %s\n", millis(), 
        name, getModeString(this->mode),getModeString(mode));
    }
    this->mode = mode;
}
