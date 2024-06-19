/*
  Author:   Mitchell C. Nelson, PhD
  Date:     March 17, 2025
  Contact:  drmcnelson@gmail.com

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1.  Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
      
  2.  Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 

 */

// ------------------------------------------------------------------
//#define DIAGNOSTICS_CPU
//#define DIAGNOSTICS_IDLER
//#define DIAGNOSTICS_SYNC
//#define DIAGNOSTICS_GATE
// Stream& dataPort = SerialUSB1;
// ------------------------------------------------------------------

#include "Arduino.h"

#include "imxrt.h"
#include "pins_arduino.h"
#define DR_INDEX    0
#define GDIR_INDEX  1
#define PSR_INDEX   2
#define ICR1_INDEX  3
#define ICR2_INDEX  4
#define IMR_INDEX   5
#define ISR_INDEX   6
#define EDGE_INDEX  7


#include <SPI.h>

#include <limits.h>

#include <ADC.h>
#include <ADC_util.h>

#include <stdlib.h>
#include <limits.h>
#include <ctype.h>

extern float tempmonGetTemp(void);

// ADC setup
ADC *adc = new ADC();

#define MAXADC 8
uint8_t adcpins[MAXADC] = { A0, A1, A2, A3, A4, A5, A6, A7 };
#define ADCPIN A0

// SPI interface to external device
const int CNVSTPin = 9;
const int SDIPin = 11;
const int SDOPin = 12;
const int CLKPin = 13;

#define SPI_SPEED 30000000
SPISettings spi_settings( SPI_SPEED, MSBFIRST, SPI_MODE0);

IMXRT_LPSPI_t *lpspi = &IMXRT_LPSPI4_S;

//*************************************************************************
// LPSPI helper functions
//*************************************************************************
static inline uint16_t get_framesz( IMXRT_LPSPI_t *port ) {
  return (port->TCR & 0x00000fff) + 1;
}
//*************************************************************************
static inline void set_framesz( IMXRT_LPSPI_t *port, uint16_t nbits ) {
  port->TCR = (port->TCR & 0xfffff000) | LPSPI_TCR_FRAMESZ(nbits-1); 
}
//*************************************************************************
static inline uint16_t transfer16( IMXRT_LPSPI_t *port, uint16_t data ) {
  port->TDR = data;                         // output 16 bit data
  while (port->RSR & LPSPI_RSR_RXEMPTY) {}  // wait while RSR fifo is empty
  return port->RDR;                         // return data read
}
//*************************************************************************

/*
uint16_t mytransfer16(uint16_t data) {
  uint32_t tcr = port().TCR;
  port().TCR = (tcr & 0xfffff000) | LPSPI_TCR_FRAMESZ(15);  // turn on 16 bit mode 
  port().TDR = data;		// output 16 bit data.
  while ((port().RSR & LPSPI_RSR_RXEMPTY)) ;	// wait while the RSR fifo is empty...
  port().TCR = tcr;	// restore back
  return port().RDR;
}
*/

// Pins for fast read/write,etc and interrupt timing
#define INPIN 5
#define OUTPIN 4
#define SPAREPIN 3

#define READMACRO (CORE_PIN5_PINREG & CORE_PIN5_BITMASK)


// Number of cycles for averaging response times
#define NKNTS 500

// CPU Cycles per time and inverse
#define CYCLES_PER_USEC (F_CPU / 1000000)
#define CYCLE_PER_NANOSEC (1.0E-9*F_CPU)
#define NANOSECS_PER_CYCLE (1.0E0/F_CPU)

// Measure elapsed time in cpu clock cycles
volatile uint32_t elapsed_cycles_holder;

inline uint32_t elapsed_cycles() {
  return (ARM_DWT_CYCCNT - elapsed_cycles_holder);
}

inline void elapsed_cycles_start() {
  elapsed_cycles_holder = ARM_DWT_CYCCNT;
}

// ------------------------------------------------------------------

const char versionstr[] = "Benchmark digital i/o";

const char authorstr[] =  "(c) 2024 by Mitchell C. Nelson, Ph.D. ";

void blink() {
  digitalWrite(13,HIGH);
  delay(1000);
  digitalWrite(13,LOW);
}

/* =====================================================================================
   Built-in ADC readout
*/
inline int fastAnalogRead( uint8_t pin ) {
  adc->adc0->startReadFast(pin);
  while ( adc->adc0->isConverting() );
  return adc->adc0->readSingle();
}

/* --------------------------------------------------------
   Send chip temperature
 */

void sendChipTemperature( unsigned int averages ){
}

/* ===================================================================
   For command processing
 */
bool testUint( char *pc, unsigned int *pu, char **next ) {
  unsigned long ul;
  char *pc1 = pc;
  ul = strtoul(pc,&pc1,0);
  if (pc1 > pc) {
    if (pu && (ul <= UINT_MAX)) *pu = ul;
    if (next) *next = pc1;
    return true;    
  }
  return false;
}

bool testKey(char *pc, const char *key, char **next) {
  int n = strlen(key);

  // skip leading white space, if any
  while (*pc && isspace(*pc)) pc++;
  
  // now test the leading characters against key
  if (*pc && !strncmp(pc,key,n)) {
    if (next) *next = pc+n;
    return true;
  }
  return false;
}

void lowercase(char *pc, int nlen) {
  while(*pc && nlen--) {
    *pc = (char) tolower((int) *pc);
    pc++;
  }
}

#define RCVLEN 256
char rcvbuffer[RCVLEN];
int readindex = 0;

int readLine( ) {

  char c;
  
  int readlen = 0;

  while (Serial.available()) {

    c = Serial.read();

    if ( c ) {

      // break at ctl-character (including \n) or semi-colon
      if (iscntrl( c ) || c == ';') {
        readlen = readindex;
        rcvbuffer[readindex] = 0;
	readindex = 0;
        return readlen;
      }

      // add c to the buffer, after leading spaces is any
      else if (readindex || !isspace(c)) {
        rcvbuffer[readindex++] = c;
      }

      // handle overflows here
      if ( readindex >= RCVLEN ) {
        Serial.println( (char *)"Error: buffer overflow" );
        readindex = 0;
	// discard the command
	return 0;
      }
    }
  }
  // not finished the line yet.
  return 0;
}

/* ===================================================================
 */
volatile uint32_t *directgpio;
volatile uint32_t directmask = 0;

inline void directAttach( uint8_t pin, void (*function)(void), int mode ) {
  directgpio = portOutputRegister(pin);
  directmask = digitalPinToBitMask(pin);
  attachInterrupt(pin, function, mode);
  attachInterruptVector(IRQ_GPIO6789, function);
}

inline uint32_t directClear( ) {
  uint32_t status = directgpio[ISR_INDEX] & directgpio[IMR_INDEX];
  if (status) {
    directgpio[ISR_INDEX] = status;
  }
  return status & directmask;
}

void directnoop() {
}

inline void directDetach(uint8_t pin) {
  attachInterrupt(pin,directnoop,RISING);
  detachInterrupt(pin);
}
  
/* ===================================================================
   We use these locally and as globals
*/
volatile uint32_t cpucycles = 0;
volatile uint32_t cpuavg = 0;
volatile uint32_t cpumax = 0;
uint32_t cpucycles_overhead = 0;

unsigned int outputpin;
unsigned int inputpin;

void timing_isr() {
  cpucycles = elapsed_cycles();
  digitalWriteFast(outputpin,LOW);

  cpuavg += cpucycles;
  if ( cpucycles > cpumax) cpumax = cpucycles;
  
  //  Serial.println("isr");
}

void timing_const_isr() {
  cpucycles = elapsed_cycles();
  digitalWriteFast(OUTPIN,LOW);
  
  cpuavg += cpucycles;
  if ( cpucycles > cpumax) cpumax = cpucycles;

  //  Serial.println("const isr");
}

void timing_direct_isr() {
  if (directClear()) {
    cpucycles = elapsed_cycles();
    digitalWriteFast(outputpin,LOW);

    cpuavg += cpucycles;
    if ( cpucycles > cpumax) cpumax = cpucycles;
  }
}

void timing_const_direct_isr() {

  /*
  uint32_t status = directgpio[ISR_INDEX] & directgpio[IMR_INDEX];
  if (status) {
    directgpio[ISR_INDEX] = status;
  }
  */
  directClear();
  
  cpucycles = elapsed_cycles();
  digitalWriteFast(OUTPIN,LOW);
  
  cpuavg += cpucycles;
  if ( cpucycles > cpumax) cpumax = cpucycles;

  //  Serial.println("const isr");
}


void measureoverhead() {
  
  uint32_t cpucycles1 = 0;
  uint32_t cpucycles2 = 0;

  cpucycles_overhead = 0;

  for (int n = 0; n < NKNTS; n++ ) {
    elapsed_cycles_start();
    cpucycles1 = elapsed_cycles();
    cpucycles2 = elapsed_cycles();
    cpucycles = cpucycles2 - cpucycles1;
    cpucycles_overhead += cpucycles;
  }
  cpucycles_overhead /= NKNTS;
  //Serial.printf("elapsed cycles timer overhead %d %.1f\n", cpucycles_overhead, ((double)cpucycles_overhead/F_CPU)*1.E9);
  Serial.printf( "#define CYCLECLOCK_OVERHEAD_CYCLES %d\n", cpucycles_overhead);
}

void sendResults( const char *s, uint32_t cavg, uint32_t cmax, uint32_t knts) {
  cavg /= knts;
  Serial.printf( "#define %s_MEASUERED_CYCLES %d\n", s, cavg);
  Serial.printf( "#define %s_MEASUERED_NANOSECS %.1f\n", s, ((double)cavg/F_CPU)*1.E9);
  Serial.printf( "#define %s_MEASUERED_CYCLES_MAX %d\n", s, cmax);
  Serial.printf( "#define %s_MEASUERED_NANOSECS_MAX %.1f\n", s, ((double)cmax/F_CPU)*1.E9);
}

/* ===================================================================
   The setup routine runs once when you press reset:
*/

void setup() {

  // initialize the digital pin as an output.
#ifdef HASLED
  pinMode(led, OUTPUT);
#endif

  pinMode(INPIN,INPUT);
  pinMode(OUTPIN,OUTPUT);

  pinMode(SPAREPIN,OUTPUT);

  // ------------------------------
  // Setup the onboard ADCs
  adc->adc0->setReference(ADC_REFERENCE::REF_3V3); 
  adc->adc0->setAveraging(1);                 // set number of averages
  adc->adc0->setResolution(12);               // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); 
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); 
  adc->adc0->wait_for_cal();  
  adc->adc0->singleMode();              // need this for the fast read

  // ------------------------------
  // SPI library takes care of the other pins
  pinMode(CNVSTPin,     OUTPUT);   
  digitalWriteFast(CNVSTPin, LOW);
  
  // ------------------------------
  Serial.begin(9600);
  delay(100);

  // Patent pending and copyright notice displayed at startup
  Serial.println( versionstr );
  Serial.println( authorstr );

#ifdef DIAGNOSTICS_CPU 
  Serial.print("F_CPU: "); Serial.print(F_CPU/1e6);  Serial.println(" MHz."); 
  //Serial.print("F_BUS: "); Serial.print(F_BUS/1e6);  Serial.println(" MHz."); 
  Serial.print("ADC_F_BUS: "); Serial.print(ADC_F_BUS/1e6); Serial.println(" MHz.");
#endif

  measureoverhead();

  blink();
  delay(1000);
  blink();

}


/* ===========================================================
 */
void loop() {

  uint16_t nlen = 0;
  char *pc;

  unsigned int pin;

  uint16_t u16tmp;

  /* --------------------------------------------------------------------
   * Command processing
   */

  if ((nlen=Serial.readBytesUntil('\n',rcvbuffer,sizeof(rcvbuffer)-1))) {
      //if ( (nlen=readLine()) ) {
    
    blink();

    lowercase(rcvbuffer,nlen);

    //Serial.println( rcvbuffer );

    /*-----------------------------------------------------------
      Put this at the top, best chance of getting to it if very busy
    */
    if ( testKey( rcvbuffer, "reboot", 0) ) {
      _reboot_Teensyduino_();
    }

    /* --------------------------------------------------------------
     */
    else if (testKey(rcvbuffer, "help",0)) {
      Serial.println( "/* -----------------------------------------------");
      Serial.println( versionstr );
      Serial.println( authorstr );
      Serial.println("  reboot         - reboots the entire board");
      Serial.println("  temperature    - report microcontroller temperature");
      Serial.println("  help           - display this message");
      Serial.println("  adc [pin]      - time analogReadFast");
      Serial.println("  macro          - time read macro");
      Serial.println("  read [npin]    - time digitalReadFast");
      Serial.println("  write [npin]   - time digitalWriteFast");
      Serial.println("  toggle [npin]  - time digitalToggleFast");
      Serial.println("  pulse [npin]   - time write high then low");
      Serial.println("  latency [npin_in npin_out] - time isr latency");
      Serial.println("  fast latency [npin_in npin_out] - time isr latency");
      Serial.println("  spi            - time spi transfers");
      Serial.println("  fast spi       - time optimized spi readout transfers");
      Serial.println("  faux spi       - time manual loop spi transfers");
      Serial.println("*/");
    }

    else if (testKey(rcvbuffer,"temp",0)) {    
	Serial.print( "CHIPTEMPERATURE " );
	Serial.println( tempmonGetTemp() );
      }

    else if (testKey( rcvbuffer, "adc", &pc)) {
       
      if (testUint(pc,&pin,NULL)) {
	cpucycles = 0;
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS; n++ ) {
	  elapsed_cycles_start();
	  u16tmp = fastAnalogRead( pin );
	  cpucycles = elapsed_cycles();
	  cpuavg += cpucycles;
	  if ( cpucycles > cpumax) cpumax = cpucycles;
	}
	cpuavg /= NKNTS;
	cpuavg -= cpucycles_overhead;
	cpumax -= cpucycles_overhead;
	Serial.printf( "//fastAnalogRead pin %d result %d\n", pin, u16tmp);
	Serial.printf( "#define FAST_ADC_CYCLES %d\n", cpuavg);
	Serial.printf( "#define FAST_ADC_NANOSECS %.1f\n", ((double)cpuavg/F_CPU)*1.E9);
	Serial.printf( "#define FAST_ADC_CYCLES_MAX %d\n", cpumax);
	Serial.printf( "#define FAST_ADC_NANOSECS_MAX %.1f\n", ((double)cpumax/F_CPU)*1.E9);
      }
      else {
	cpucycles = 0;
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS; n++ ) {
	  elapsed_cycles_start();
	  u16tmp = fastAnalogRead( ADCPIN );
	  cpucycles = elapsed_cycles();
	  cpuavg += cpucycles;
	  if ( cpucycles > cpumax) cpumax = cpucycles;
	}
	sendResults( "FAST_ADC", cpuavg, cpumax, NKNTS);
      }
    }
    
    else if (testKey( rcvbuffer, "read", &pc)) {
       
      if (testUint(pc,&pin,NULL)) {
	cpucycles = 0;
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS; n++ ) {
	  elapsed_cycles_start();
	  u16tmp = digitalReadFast( pin );
	  cpucycles = elapsed_cycles();
	  cpuavg += cpucycles;
	  if ( cpucycles > cpumax) cpumax = cpucycles;
	}
	sendResults( "DIGITAL_READ_FAST", cpuavg, cpumax, NKNTS);
      }
      else {
	cpucycles = 0;
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS; n++ ) {
	  elapsed_cycles_start();
	  u16tmp = digitalReadFast( INPIN );
	  cpucycles = elapsed_cycles();
	  cpuavg += cpucycles;
	  if ( cpucycles > cpumax) cpumax = cpucycles;
	}
	sendResults( "DIGITAL_READ_FAST_CONST", cpuavg, cpumax, NKNTS);
      }
    }

    else if (testKey( rcvbuffer, "macro", &pc)) {
	cpucycles = 0;
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS; n++ ) {
	  elapsed_cycles_start();
	  u16tmp = READMACRO;
	  cpucycles = elapsed_cycles();
	  cpuavg += cpucycles;
	  if ( cpucycles > cpumax) cpumax = cpucycles;
	}
	sendResults( "READMACRO", cpuavg, cpumax, NKNTS);
    }

    else if (testKey( rcvbuffer, "write", &pc)) {
       
      if (testUint(pc,&pin,NULL)) {
	cpucycles = 0;
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS/2; n++ ) {
	  elapsed_cycles_start();
	  digitalWriteFast( pin, HIGH );
	  cpucycles = elapsed_cycles();
	  cpuavg += cpucycles;
	  if ( cpucycles > cpumax) cpumax = cpucycles;

	  delayNanoseconds(100);
	  
	  elapsed_cycles_start();
	  digitalWriteFast( pin, LOW );
	  cpucycles = elapsed_cycles();
	  cpuavg += cpucycles;
	  if ( cpucycles > cpumax) cpumax = cpucycles;
	}
	sendResults( "DIGITAL_WRITE_FAST", cpuavg, cpumax, NKNTS);
      }
      else {
	cpucycles = 0;
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS/2; n++ ) {
	  elapsed_cycles_start();
	  digitalWriteFast( OUTPIN, HIGH );
	  cpucycles = elapsed_cycles();
	  cpuavg += cpucycles;
	  if ( cpucycles > cpumax) cpumax = cpucycles;

	  delayNanoseconds(100);
	  
	  elapsed_cycles_start();
	  digitalWriteFast( OUTPIN, LOW );
	  cpucycles = elapsed_cycles();
	  cpuavg += cpucycles;
	  if ( cpucycles > cpumax) cpumax = cpucycles;
	}
	sendResults( "DIGITAL_WRITE_FAST_CONST", cpuavg, cpumax, NKNTS);
      }
    }

    else if (testKey( rcvbuffer, "pulse", &pc)) {
       
      if (testUint(pc,&pin,NULL)) {
	cpucycles = 0;
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS; n++ ) {
	  elapsed_cycles_start();
	  digitalWriteFast( pin, HIGH );
	  digitalWriteFast( pin, LOW );
	  cpucycles = elapsed_cycles();
	  cpuavg += cpucycles;
	  if ( cpucycles > cpumax) cpumax = cpucycles;
	}
	sendResults( "DIGITAL_PULSE_FAST", cpuavg, cpumax, NKNTS);
      }
      else {
	cpucycles = 0;
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS; n++ ) {
	  elapsed_cycles_start();
	  digitalWriteFast( OUTPIN, HIGH );
	  digitalWriteFast( OUTPIN, LOW );
	  cpucycles = elapsed_cycles();
	  cpuavg += cpucycles;
	  if ( cpucycles > cpumax) cpumax = cpucycles;
	}
	sendResults( "DIGITAL_PULSE_FAST_CONST", cpuavg, cpumax, NKNTS);
      }
    }

    else if (testKey( rcvbuffer, "toggle", &pc)) {
       
      if (testUint(pc,&pin,NULL)) {
	cpucycles = 0;
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS; n++ ) {

	  elapsed_cycles_start();
	  digitalToggleFast( pin );
	  cpucycles = elapsed_cycles();
	  cpuavg += cpucycles;
	  if ( cpucycles > cpumax) cpumax = cpucycles;

	}
	sendResults( "DIGITAL_TOGGLE_FAST", cpuavg, cpumax, NKNTS);
      }
      else {
	cpucycles = 0;
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS; n++ ) {

	  elapsed_cycles_start();
	  digitalToggleFast( OUTPIN );
	  cpucycles = elapsed_cycles();
	  cpuavg += cpucycles;
	  if ( cpucycles > cpumax) cpumax = cpucycles;

	}
	sendResults( "DIGITAL_TOGGLE_FAST_CONST", cpuavg, cpumax, NKNTS);
      }
    }


    else if (testKey( rcvbuffer, "latency", &pc)) {


      if (testUint(pc,&inputpin,&pc)&&testUint(pc,&outputpin,NULL)) {

	pinMode(inputpin,INPUT);
	pinMode(outputpin,OUTPUT);
	digitalWriteFast( outputpin, LOW );

	attachInterrupt(digitalPinToInterrupt(inputpin), timing_isr, RISING);

	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS; n++ ) {

	  cpucycles = 0;
	  elapsed_cycles_start();
	  digitalWriteFast( outputpin, HIGH );

	  delayMicroseconds(1);
	  if (!cpucycles) {
	    Serial.print("isr did not run");\
	    Serial.println(n);\
	    break;
	  }
	}

	detachInterrupt( digitalPinToInterrupt(inputpin));
	
	sendResults( "LATENCY", cpuavg, cpumax, NKNTS);
	
      }
      else {

	pinMode(INPIN,INPUT);
	pinMode(OUTPIN,OUTPUT);
	digitalWriteFast( OUTPIN, LOW );

	attachInterrupt(digitalPinToInterrupt(INPIN), timing_const_isr, RISING);
	
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS; n++ ) {
	  
	  cpucycles = 0;
	  elapsed_cycles_start();
	  digitalWriteFast(OUTPIN,HIGH);

	  delayMicroseconds(1);
	  if (!cpucycles) {
	    Serial.print("isr did not run");\
	    Serial.println(n);\
	    break;
	  }
	}
	
	detachInterrupt( digitalPinToInterrupt(INPIN));

	sendResults( "LATENCY_CONST", cpuavg, cpumax, NKNTS);
      }
    }
    
    else if (testKey( rcvbuffer, "fast latency", &pc)) {


      if (testUint(pc,&inputpin,&pc)&&testUint(pc,&outputpin,NULL)) {

	pinMode(inputpin,INPUT);
	pinMode(outputpin,OUTPUT);
	digitalWriteFast( outputpin, LOW );

	directAttach(digitalPinToInterrupt(inputpin), timing_direct_isr, RISING);
	/*	     
	attachInterrupt(digitalPinToInterrupt(inputpin), timing_fast_isr, RISING);
	directconnect( digitalPinToInterrupt(inputpin), timing_fast_isr );
	*/
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS; n++ ) {

	  cpucycles = 0;
	  elapsed_cycles_start();
	  digitalWriteFast( outputpin, HIGH );

	  delayMicroseconds(1);
	  if (!cpucycles) {
	    Serial.print("isr did not run");\
	    Serial.println(n);\
	    break;
	  }
	}


	directDetach(digitalPinToInterrupt(inputpin));
	
	sendResults( "LATENCY_DIRECT", cpuavg, cpumax, NKNTS);
	
      }
      else {

	pinMode(INPIN,INPUT);
	pinMode(OUTPIN,OUTPUT);
	digitalWriteFast( OUTPIN, LOW );

	directAttach(digitalPinToInterrupt(inputpin), timing_const_direct_isr, RISING);
	/*
	attachInterrupt(digitalPinToInterrupt(INPIN), timing_const_fast_isr, RISING);
	directconnect( digitalPinToInterrupt(INPIN), timing_const_fast_isr );
	*/
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS; n++ ) {
	  
	  cpucycles = 0;
	  elapsed_cycles_start();
	  digitalWriteFast(OUTPIN,HIGH);

	  delayMicroseconds(1);
	  if (!cpucycles) {
	    Serial.print("isr did not run");\
	    Serial.println(n);\
	    break;
	  }
	}
	
	directDetach(digitalPinToInterrupt(INPIN));

	sendResults( "LATENCY_DIRECT_CONST", cpuavg, cpumax, NKNTS);
      }
    }

    else if (testKey( rcvbuffer, "spi", &pc)) {

      uint16_t data[NKNTS];

      SPI.begin();
      SPI.beginTransaction(spi_settings);

      cpuavg = 0;
      cpumax = 0;
      for (int n = 0; n < NKNTS; n++ ) {

	digitalWriteFast(CNVSTPin,HIGH);
	delayNanoseconds(700);
	digitalWriteFast(CNVSTPin,LOW);
	
	elapsed_cycles_start();
	
	digitalWriteFast(SPAREPIN,HIGH);
	data[n] = SPI.transfer16(0xFFFF);
	digitalWriteFast(SPAREPIN,LOW);

	cpucycles = elapsed_cycles();
	cpuavg += cpucycles;
	if ( cpucycles > cpumax) cpumax = cpucycles;

      }
      sendResults( "SPI_transfer16", cpuavg, cpumax, NKNTS);
	      
      SPI.endTransaction();
      SPI.end();

    }

    else if (testKey( rcvbuffer, "fast spi", &pc)) {
      uint16_t saved_framesz;
      uint16_t data[NKNTS];

      cpuavg = 0;
      cpumax = 0;
      
      SPI.begin();
      SPI.beginTransaction(spi_settings);
      saved_framesz = get_framesz( lpspi );
      set_framesz( lpspi, 16 );

      for (uint16_t i=0; i<NKNTS; i++){ 

	digitalWriteFast(CNVSTPin,HIGH);
	delayNanoseconds( 700 );
	digitalWriteFast(CNVSTPin,LOW);

	elapsed_cycles_start();

	digitalWriteFast(SPAREPIN,HIGH);
	data[i] = transfer16( lpspi, 0xFFFF );
	digitalWriteFast(SPAREPIN,LOW);

	cpucycles = elapsed_cycles();
	cpuavg += cpucycles;
	if ( cpucycles > cpumax) cpumax = cpucycles;
      }

      set_framesz( lpspi, saved_framesz ); // restore original
      SPI.endTransaction();

      sendResults( "SPI_fast_transfer16", cpuavg, cpumax, NKNTS);
    }
  
    else if (testKey( rcvbuffer, "faux spi", &pc)) {
      //uint32_t cyclesnext, cyclesnow;
      int d[32] = {0};
      int dummy;
      
      pinMode(CNVSTPin,OUTPUT);
      pinMode(SDIPin,INPUT);
      pinMode(SDOPin,OUTPUT);
      pinMode(CLKPin,OUTPUT);

      digitalWriteFast(CNVSTPin,LOW);
      digitalWriteFast(CLKPin,LOW);
      delayNanoseconds(1000);
      
      digitalWriteFast(CNVSTPin,HIGH);
      delayNanoseconds(700);
      digitalWriteFast(CNVSTPin,LOW);
      
      digitalWriteFast(SDOPin,HIGH);
      //cyclesnext = ARM_DWT_CYCCNT + 10;
      for (int n = 0; n < 16; n++ ) {

	/*
	while( (cyclesnow=ARM_DWT_CYCCNT) < cyclesnext);
	cyclesnext = cyclesnow + 10;
	*/
	digitalWriteFast(CLKPin,HIGH);

	d[n] = digitalReadFast(SDIPin);

	/*
	while( (cyclesnow=ARM_DWT_CYCCNT) < cyclesnext);
	cyclesnext = cyclesnow + 10;
	*/
	digitalWriteFast(CLKPin,LOW);
	dummy = digitalReadFast(SDIPin);

      }
      Serial.print("Result ");
      for (int n = 0; n < 16; n++ ) {
	Serial.print( d[n]&1 );
      }
      Serial.println("");
    }

    else {
      Serial.print( "Error: command not recognized: " );
      Serial.println( rcvbuffer );
    }

    
    // Indicate that we processed this message
    nlen = 0;

    //Serial.println( "DONE" );
  }

  delay(1);
}
