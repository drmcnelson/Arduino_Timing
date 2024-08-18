# Arduino_Timing
Measure interrupt latency, jitter and timing for various digital and analog i/o operations that you may need
if you are developing an embedded instrument or control system.

The output is in the syntax of a header file for C or C++.

The code includes section that uses the standard Arduino API as well as specific code for the Teensy 4.x and
the Arduino UNO R4.

For the R4, you will want to use my version of the SPI library, this version adds true 16-bit transfers and
enhanced loop friendly transfer comprising three calls  transfer16_setup(), transfer16_transfer(), and 
transfer16_cleanup()). 

Look for the files SPI.cpp and SPI.h in my fork of the Arduino Renesas support package.

In the sub-directory  Python, you will find a Python script  SimpleDataLOgger.py that can be used as a class
library or a utility to issue commands and collects results.  There are scripts that you can modify to suit your
needs.  From the program's cli, the command to runa script is @scriptfilespec.   To record output to a logfile,
run the program with the switch --logfile logfilespec

The subdirectory Images has some oscilloscope images collected using the program.
