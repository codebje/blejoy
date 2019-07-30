Quick Design Notes

  - ESP32 dev board runs BLE stack as gamepad device
  - perf-board takes connections for all inputs
  - battery board provides power, not sure how to control on/off yet

ESP32 interrupt driven to set notification on BLE comms task, multiple
notifications are okay, just send current state whenever anything's changed.

Pin assignment:

GPIO16..33 is ok for digital input. Exposed on the DOIT v1 board is:
    16, 17, 18, 19, 21, 22  - right side, six main buttons
    27, 26, 25, 33          - left side, joystick
    32, 23                  - left, right sides, start/select buttons

Joystick cable from button of joystick to top:

 1. GND
 2. Left        GPIO 27
 3. Right       GPIO 26
 4. Down        GPIO 25
 5. Up          GPIO 33

Buttons do not care which side is GND. Configure inputs with PULLUP resistors
and they'll do just fine.