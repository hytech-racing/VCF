## CAN interface

outputs:
- pedal data CAN packet (`0xC0`):
    - status bits: (8 bits)
        - `bool accel_implausible`
            - accel pedal value is out of range
        - `bool brake_implausible`
            - brake pedal value is out of range
        - `bool brake_pedal_active`
        - `bool accel_pedal_active`
        - `bool mech_brake_active`
            - brake pedal has reached zone in which the mechanical brake (the physical calipers) have started engaging
        - `bool brake_and_accel_pressed_implausibility`
        - `bool implausibility_exceeded_duration`
            - an implausibility been present for longer than allowed (>200ms by rules)
                - __note__: we should guard this to be over 180ms or some threshold below 200ms to allow for transmission delay to stay within rules as this is now being reacted to by the VCR
    - data (32 bits):
        - `brake_pedal` (16 bit unsigned) -> mapped between 0 and 1 (65,535)
        - `accel_pedal` (16 bit unsigned) -> mapped between 0 and 1 (65,535)
- `FRONT_SUSPENSION` (`0xED`)
    - `uint16_t rl_load_cell`
    - `uint16_t rr_load_cell`
    - `uint16_t rl_shock_pot`
    - `uint16_t rr_shock_pot`
