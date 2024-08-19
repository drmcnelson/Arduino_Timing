# Arduino_Timing
Measure interrupt latency, jitter and timing for various digital and analog i/o operations that you may need
if you are developing an embedded instrument or control system.

The output is in the syntax of a header file for C or C++.

The code includes sections that use the standard Arduino API as well as specific code for the Teensy 4.x and
the Arduino UNO R4.

The Teensy4 is extremely fast, you can see oscilloscope images for the timing in the Images subdirectory

The Uno R4 is a bit slower and there are some issues in some of their libraries.  If you want to run on an
UNO R4, you need to use the upgraded SPI library which you can download from here https://github.com/drmcnelson/Arduino_UNO_R4_SPI_Speedup

The upgrade for the UNO R4 SPI library adds true 16-bit transfers and an enhanced loop friendly API with reduced overhead for repeated transfers.
True 16 bit transfers are already built into the Teensy 4 SPI libraries.

In the sub-directory  Python, you will find a Python script  SimpleDataLOgger.py that can be used as a class
library or a utility to issue commands and collects results.  There are scripts that you can modify to suit your
needs.  From the program's cli, the command to runa script is @scriptfilespec.   To record output to a logfile,
run the program with the switch --logfile logfilespec

The Images subdirectory has oscilloscope images for several other tests as well as the SPI.
