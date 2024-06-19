# Arduino_Timing
Measure interrupt latency, jitter and timing for various digital and analog i/o operations.

Microcontrollers (MCU) are increasingly being used to operate devices require tight timing, in a sense displacing the FPGA from some of its traditional domain.
If you are using your Arduino or Teensy, or similar, in this way, you may be interested in timing and jitter for various functions in your platform.
This is an Arduino sketch that measures and reports interrupt latency, and timing required to peforma various digital and analog i/o functions.
The output is in a format that lets you conveniently include the results as macros in a header file.
The code has been tested on a Teensy 4.0.  Please let me know if you would like to contribute modifications for other boards.
