# Code

### Required

    gcc
    arm-non-eabi-gcc
    cMake

### Mac installation process

Install brew: https://brew.sh/index_de, then run:

    brew install gcc-arm-embedded
    brew install cmake

## Troubleshooting

Try with arduion mega with exmple code from TMC2209 library each stepper driver

If not working with Arudion, test after taking each of those stepps:
    - Check each connection
    - Make sure tx and rx are correct
    - Use different stepper driver
    - Use different cable to computer
    - Replace resistor
    - Try on different computer
    
If it works with the arduino, connections are right, try with pico again, but with commit 97471ef
If it works on that commit but not currently the code is probably wrong.
If it does not work on that commit try:
    - Replacing the pico
    - Use different cable to computer
    - Use different computer
