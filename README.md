# Arduino_Timing
Measure interrupt latency, jitter and timing for various digital and analog i/o operations.

Microcontrollers (MCU) are increasingly being used in applications where tiking is criticial, in effect displacing the FPGA from some of its traditional domain.
If you are using your Arduino or Teensy, or similar, in this way, you may be interested in measuring timing and jitter for various functions including digital read, write, toggle, SPI transfers, analog i/o.
This is an Arduino sketch measures and reports timings and generates output in the format of macros that you can include in a header file.
The code has been tested on a Teensy 4.0 and UNO R4.  Please let me know if you would like to contribute modifications for other boards.
