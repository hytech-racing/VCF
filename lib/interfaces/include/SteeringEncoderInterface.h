#ifndef STEERING_ENCODER_INTERFACE_H
#define STEERING_ENCODER_INTERFACE_H

enum class SteeringEncoderStatus_e
{
    STEERING_ENCODER_NOMINAL = 0,
    STEERING_ENCODER_ERROR = 1,
};

struct EncoderErrorFlags_s
{
    bool generalError             = false;  // Position data invalid (bit=0 means error)
    bool generalWarning           = false;  // Position data valid, operating conditons near limits (bit=0 means warning)
    bool noData                   = false;  // No data received
};

struct SteeringEncoderConversion_s  // This is the final struct I would like to send across the car. What's seen in foxglove?
{
    float angle = 0.0f;
    int raw = 0;
    SteeringEncoderStatus_e status = SteeringEncoderStatus_e::STEERING_ENCODER_NOMINAL;
    EncoderErrorFlags_s errors;
};

class SteeringEncoderInterface
{
public:
// Functions
    /// @brief Commands the underlying steering sensor to sample and hold the result.
    virtual void sample() = 0;
    /// @brief Calculate steering angle and whether result is in sensor's defined bounds. DOES NOT SAMPLE.
    /// @return Calculated steering angle in degrees.
    virtual SteeringEncoderConversion_s convert() = 0;
};

#endif /* STEERING_ENCODER_INTERFACE_H */
