# Bugs and their fixes

## Stepper motor not moving when powered on
- Check whether the `DRV_wakeup` function has been run in the stepperFn task. 
    - The power draw on the Disc is around 20mA-40mA, but the Disc draws about 350mA when a stepper motor is "live" and drawing non-negligible amounts of current. 

## Stepper motor meant to move "smoothly" but jitters instead
- Check and see if an `osDelay` is present in the stepperFn task, since that can seemingly delay the pulses themselves
