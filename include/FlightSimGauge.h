#include <Arduino.h>
#include <AccelStepper.h>

class FlightSimGauge {
    public:
        enum FlightSimGaugeMode {
            GAUGE_OPERATING,        
            GAUGE_HOMING_WAIT_PRESS,
            GAUGE_HOMING_MANUAL,
            GAUGE_HOMING_WAIT_RELEASE,
            GAUGE_HOMING_AUTO,
            GAUGE_HOMING_LIMIT
        };

        FlightSimGauge(const char *gaugeName, AccelStepper &stepper);
        void setManualHomingPinAndSpeed(int16_t pinNumber, uint8_t activeLevel, int32_t homingSpeed);
        void setManualHomingPinTiming(uint32_t pressDelay, uint32_t releaseDelay);
        void setAutoHomingPinAndSpeed(int16_t pinNumber, uint8_t activeLevel, int32_t homingSpeed);
        void setLimitHoming(int32_t steps);
        void setHomingPosition(int32_t position) { this->homingPosition = homingPosition; }
        void setTargetPosition(int32_t targetPosition) { stepper.moveTo(targetPosition); };

        void loop();

        void setDebug(bool debugMode, uint32_t timer=500) { this->debugMode = debugMode; debugInterval=timer;  };
        FlightSimGaugeMode getMode() { return mode; };

    private:
        const char *name;
        AccelStepper &stepper;
        FlightSimGaugeMode mode;
        elapsedMillis timer;
        elapsedMillis debugTimer;
        uint32_t debugInterval;
        bool debugMode;
        int16_t manualHomingPin;
        int16_t autoHomingPin;
        bool isLimitHoming;
        int32_t homingSpeed;
        uint8_t homingLevel;
        int32_t homingPosition;
        uint32_t manualHomingPressDelay;
        uint32_t manualHomingReleaseDelay;
        int32_t limitHomingPosition;

        const char *getModeString(FlightSimGaugeMode mode);
        void setMode(FlightSimGaugeMode mode);
};
