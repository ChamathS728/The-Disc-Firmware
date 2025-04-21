# Bugs and their fixes

## Stepper motor not moving when powered on
- Check whether the `DRV_wakeup` function has been run in the stepperFn task. 
    - The power draw on the Disc is around 20mA-40mA, but the Disc draws about 350mA when a stepper motor is "live" and drawing non-negligible amounts of current. 

## Stepper motor meant to move "smoothly" but jitters instead
- Check and see if an `osDelay` is present in the stepperFn task, since that can seemingly delay the pulses themselves


## Packet structure cast into a uint8_t buffer??
- For the timestamp (uint32_t), the data is placed LSB first
    - Eg: Suppose a timestamp is 3772184
    - When casted into a buffer, the number is split into 4 bytes
        - Byte 0: 24    - corresponds to the 2nd entry in the buffer
        - Byte 1: 143   - corresponds to the 3rd entry in the buffer
        - Byte 2: 57    - corresponds to the 4th entry in the buffer
        - Byte 3: 0     - corresponds to the 5th entry in the buffer
    - Reconstructing this:
        - $(0 \times 2^{24}) + (57 * 2^{16}) + (143 * 2^8) + (24 * 2^0) = 3772184$