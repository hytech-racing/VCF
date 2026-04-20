#ifndef STEERING_ENCODER_INTERFACE_H
#define STEERING_ENCODER_INTERFACE_H

enum class SteeringEncoderStatus_e
{
    NOMINAL = 0,
    ERROR = 1,
};

/// @brief Error and warning flags from the encoder.
/// @note Specific encoders may populate only a subset of these flags.
struct EncoderErrorFlags_s
{
    bool dataInvalid              = false;
    bool operatingLimit           = false;
    bool noData                   = false;
};

/// @brief Complete steering angle measurement with status and error flags.
/// @note This struct is transmitted across the CAN bus to other vehicle systems.
struct SteeringEncoderReading_s
{
    float angle = 0.0f;
    int rawValue = 0;
    SteeringEncoderStatus_e status = SteeringEncoderStatus_e::NOMINAL;
    EncoderErrorFlags_s errors;
};

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
