#ifndef STEERING_ENCODER_INTERFACE_H
#define STEERING_ENCODER_INTERFACE_H
#include "SharedFirmwareTypes.h"

class SteeringEncoderInterface
{
public:
// Functions
    /// @brief Reads current position from the physical sensor and stores the result.
    virtual void sample() = 0;
    /// @brief Returns the most recently stored reading.
    /// @note DOES NOT SAMPLE/COMMUNICATE WITH SENSOR
    /// @return Struct containing angle, raw value, status, and error flags
    virtual SteeringEncoderReading_s getLastReading() = 0;
};

#endif /* STEERING_ENCODER_INTERFACE_H */
