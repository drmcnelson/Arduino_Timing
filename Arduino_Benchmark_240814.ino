/*
  Author:   Mitchell C. Nelson, PhD
  Date:     March 17, 2024
  Contact:  drmcnelson@gmail.com

  Measures and reports timings various basic operations.

  Original version             For Teensy 4.0 and 3.2
  
  Revision  August 14, 2024    Include support for the UNO R4
  

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
#include <stdlib.h>
#include <limits.h>
#include <ctype.h>

const char versionstr[] = "Benchmark digital i/o";

const char authorstr[] =  "(c) 2024 by Mitchell C. Nelson, Ph.D. ";

// Number of cycles for averaging response times
#define NKNTS 500

// CPU Cycles per time and inverse
#define CYCLES_PER_USEC (F_CPU / 1000000)
#define CYCLE_PER_NANOSEC (1.0E-9*F_CPU)
#define SECS_PER_CYCLE (1./F_CPU)
#define NANOSECS_PER_CYCLE (1.0E-9/F_CPU)

#define NANOSECS_TO_CYCLES(n) (((n)*CYCLES_PER_USEC)/1000)

volatile uint32_t cpucycles = 0;
volatile uint32_t cpucycles2 = 0;
volatile uint32_t cpuavg = 0;
volatile uint32_t cpumax = 0;
uint32_t cpucycles_overhead = 0;

unsigned int outputpin;
unsigned int inputpin;
unsigned int timertest_countdown = 0;

int serial_printf(const char *format, ...)
{
  char buffer[128] = {0};
  int n = 0;

  va_list args;
  va_start(args, format);
  n = vsprintf(buffer, format, args);
  va_end(args);

  Serial.print(buffer);

  return n;
}

// ==========================================================
// Identify our platform

#if defined(ARDUINO_TEENSY41)||defined(ARDUINO_TEENSY40)
#define IS_TEENSY

#elif defined(ARDUINO_TEENSY36)||defined(ARDUINO_TEENSY35)||defined(ARDUINO_TEENSY32)
#define IS_TEENSY

#elif defined(ARDUINO_UNOR4_MINIMA) || defined(ARDUINO_UNOR4_WIFI)
#define IS_ARDUINO_UNO_R4

#else
#error "not recognized as a supported board"

#endif

// ==========================================================
// Specific to the Teensy 4.0, 3.2

#ifdef IS_TEENSY

#include <digitalWriteFast.h>
#define DIGITALREAD(a) digitalReadFast(a)
#define DIGITALWRITE(a,b) digitalWriteFast(a,b)
#define DIGITALTOGGLE(a) digitalToggleFast(a)

inline void _rebootFunc() {
  _reboot_Teensyduino_();
}

// Pins for fast read/write,etc and interrupt timing
#define INPIN 5
#define OUTPIN 4
#define SPAREPIN 3

#define READMACRO (CORE_PIN5_PINREG & CORE_PIN5_BITMASK)

// SPI interface to external device on T4 controller
const int CNVSTPin = 10;
const int SDIPin = 11;
const int SDOPin = 12;
const int CLKPin = 13;

#define MAXADC 8
uint8_t adcpins[MAXADC] = { A0, A1, A2, A3, A4, A5, A6, A7 };
#define ADCPIN A0

void blink() {
  digitalWrite(13,HIGH);
  delay(1000);
  digitalWrite(13,LOW);
}

#define HAS_CPU_TEMPERATURE
extern float tempmonGetTemp(void);

inline float getCpuTemperature() {
  return tempmonGetTemp();
}

// Measure elapsed time in cpu clock cycles
volatile uint32_t elapsed_cycles_holder;

inline uint32_t elapsed_cycles() {
  return (ARM_DWT_CYCCNT - elapsed_cycles_holder);
}

inline void elapsed_cycles_start() {
  elapsed_cycles_holder = ARM_DWT_CYCCNT;
}

void setupCYCCNT() {
  elapsed_cycles_holder = ARM_DWT_CYCCNT;
}


#define DELAYNANOSECONDS(a) nanoDelay(a)
inline void nanoDelay(uint32_t nsecs)
{
  uint32_t volatile start_cycles = ARM_DWT_CYCCNT;
  uint32_t volatile wait_cycles = NANOSECS_TO_CYCLES(nsecs);
  while ((ARM_DWT_CYCCNT-start_cycles)<wait_cycles);
}



// -------------------------------------------
// ADC library specific for the Teensy 4.0, 3.2, etc
#include <ADC.h>
#include <ADC_util.h>

ADC *adc = new ADC();

void adcSetup() {
  // Setup the onboard ADCs
  adc->adc0->setReference(ADC_REFERENCE::REF_3V3); 
  adc->adc0->setAveraging(1);                 // set number of averages
  adc->adc0->setResolution(12);               // set bits of resolution
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED); 
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); 
  adc->adc0->wait_for_cal();  
  adc->adc0->singleMode();              // need this for the fast read
}

inline int fastAnalogRead( uint8_t pin ) {
  adc->adc0->startReadFast(pin);
  while ( adc->adc0->isConverting() );
  return adc->adc0->readSingle();
}

// -------------------------------------------
// SPI Specific for the Teensy 4.0, 3.2
#include <SPI.h>

#define SPI_SPEED 30000000
SPISettings spi_settings( SPI_SPEED, MSBFIRST, SPI_MODE0);

#define USE_SPI_SPEEDUP

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

IMXRT_LPSPI_t *lpspi = &IMXRT_LPSPI4_S;

static inline uint16_t get_framesz( IMXRT_LPSPI_t *port ) {
  return (port->TCR & 0x00000fff) + 1;
}

static inline void set_framesz( IMXRT_LPSPI_t *port, uint16_t nbits ) {
  port->TCR = (port->TCR & 0xfffff000) | LPSPI_TCR_FRAMESZ(nbits-1); 
}
static inline uint16_t transfer16( IMXRT_LPSPI_t *port, uint16_t data ) {
  port->TDR = data;                         // output 16 bit data
  while (port->RSR & LPSPI_RSR_RXEMPTY) {}  // wait while RSR fifo is empty
  return port->RDR;                         // return data read
}

// -------------------------------------------
// Accelerated ISR handling specific to ARM

#define USE_INTERRUPT_SPEEDUP

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

void timing_direct_isr() {
  if (directClear()) {
    cpucycles = elapsed_cycles();
    DIGITALWRITE(outputpin,LOW);
    cpuavg += cpucycles;
    if ( cpucycles > cpumax) cpumax = cpucycles;
  }
}

void timing_const_direct_isr() {
  directClear();
  cpucycles = elapsed_cycles();
  DIGITALWRITE(OUTPIN,LOW);  
  cpuavg += cpucycles;
  if ( cpucycles > cpumax) cpumax = cpucycles;

  //  Serial.println("const isr");
}

// ----------------------------------------
IntervalTimer mytimer;

void timertest_isr() {
  if (!cpucycles) {
    cpucycles = elapsed_cycles();
  }
  DIGITALTOGGLE(OUTPIN);
  if (timertest_countdown) {
    timertest_countdown--;
  }
  if (!timertest_countdown) {
    cpucycles2 = elapsed_cycles();
    mytimer.end();
  }
}

void timerstart( unsigned int usecs) {
  elapsed_cycles();
  mytimer.begin(timertest_isr,usecs);
}

// ==========================================================
#elif defined(IS_ARDUINO_UNO_R4)

//#include <digitalWriteFast.h>
#define DIGITALREAD(a) digitalRead(a)
#define DIGITALWRITE(a,b) digitalWrite(a,b)
#define DIGITALTOGGLE(a) digitalWrite(a,!digitalRead(a))

#include <TimerOne.h>


// Pins for fast read/write,etc and interrupt timing
#define INPIN 5
#define OUTPIN 4
#define SPAREPIN 3

// SPI interface to external device
const int CNVSTPin = 10;
const int SDIPin = 11;
const int SDOPin = 12;
const int CLKPin = 13;

// Uno has 6 analog inputs
#define MAXADC 6
uint8_t adcpins[MAXADC] = { A0, A1, A2, A3, A4, A5 };
#define ADCPIN A0

void adcSetup() {
  analogReadResolution(14);
}

// -------------------------------------------
#include <SPI.h>

//#define SPI_SPEED 20000000
#define SPI_SPEED 12000000
SPISettings spi_settings( SPI_SPEED, MSBFIRST, SPI_MODE0);

void blink() {
  digitalWrite(13,HIGH);
  delay(1000);
  digitalWrite(13,LOW);
}

void(*_rebootFunc)(void)=0;

void setupCYCCNT() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  ITM->LAR = 0xc5acce55;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t elapsed_cycles_holder = 0;

#define ARM_DWT_CYCCNT DWT->CYCCNT

inline uint32_t elapsed_cycles() {
  return (DWT->CYCCNT - elapsed_cycles_holder);
}

inline void elapsed_cycles_start() {
  elapsed_cycles_holder = DWT->CYCCNT;
}

#define DELAYNANOSECONDS(a) nanoDelay(a)

inline void nanoDelay(uint32_t nsecs)
{
  uint32_t start_cycles = DWT->CYCCNT;
  uint32_t wait_cycles = (nsecs * (F_CPU/1000))/1000000;
  while ((DWT->CYCCNT-start_cycles)<wait_cycles);
}

void timertest_isr() {
  if (!cpucycles) {
    cpucycles = elapsed_cycles();
  }
  DIGITALTOGGLE(OUTPIN);
  if (timertest_countdown) {
    timertest_countdown--;
  }
  if (!timertest_countdown) {
    cpucycles2 = elapsed_cycles();
    Timer1.stop();
    Timer1.detachInterrupt();
  }
}

bool timer_initialized = false;

void timerstart( unsigned int usecs) {  
  if (!timer_initialized) {
    Timer1.initialize(100000);
    delay(1000);
    Timer1.stop();
    timer_initialized = true;
  }

  timertest_countdown = 10;        
  Timer1.setPeriod(usecs);
  Timer1.attachInterrupt(timertest_isr);

  elapsed_cycles();
  Timer1.start();
}


#endif

void timing_isr() {
  cpucycles = elapsed_cycles();
  DIGITALWRITE(outputpin,LOW);
  cpuavg += cpucycles;
  if ( cpucycles > cpumax) cpumax = cpucycles;
}

void timing_const_isr() {
  cpucycles = elapsed_cycles();
  DIGITALWRITE(OUTPIN,LOW);
  cpuavg += cpucycles;
  if ( cpucycles > cpumax) cpumax = cpucycles;
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
  //serial_printf("elapsed cycles timer overhead %d %.1f\n", cpucycles_overhead, ((double)cpucycles_overhead/F_CPU)*1.E9);
  serial_printf( "#define CYCLECLOCK_OVERHEAD_CYCLES %d\n", cpucycles_overhead);
}

void sendResults( const char *s, uint32_t cavg, uint32_t cmax, uint32_t knts) {
  cavg /= knts;
  serial_printf( "#define %s_MEASUERED_CYCLES %d\n", s, cavg);
  serial_printf( "#define %s_MEASUERED_NANOSECS %.1f\n", s, ((double)cavg/F_CPU)*1.E9);
  serial_printf( "#define %s_MEASUERED_CYCLES_MAX %d\n", s, cmax);
  serial_printf( "#define %s_MEASUERED_NANOSECS_MAX %.1f\n", s, ((double)cmax/F_CPU)*1.E9);
}

/* ===================================================================
   For command processing
 */
bool testUint( char *pc, unsigned int *pu, char **next ) {
  unsigned long ul;
  char *pc1 = pc;
  ul = strtoul(pc,&pc1,0);
  if ((pc1 > pc)&&(ul <= UINT_MAX)) {
    if (pu) *pu = ul;
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

      // add c to the buffer, after leading spaces if any
      else if (readindex || !isspace(c)) {
        rcvbuffer[readindex++] = c;
        // set the next byte to zero, always, until eob
        if (readindex<RCVLEN) rcvbuffer[readindex] = 0;
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
   The setup routine runs once when you press reset:
*/

void setup() {

#ifdef HASLED
  pinMode(led, OUTPUT);
  blink();
  delay(1000);
  blink();
  delay(1000);
#endif

  // initialize the digital pins
  pinMode(INPIN,INPUT);
  pinMode(OUTPIN,OUTPUT);
  pinMode(SPAREPIN,OUTPUT);

  // ------------------------------
  // Setup the MCU Cycle Counter
  setupCYCCNT();
  
  // ------------------------------
  // Setup the onboard ADCs
  adcSetup();

  // ------------------------------
  // SPI library takes care of the other pins
  pinMode(CNVSTPin,     OUTPUT);   
  DIGITALWRITE(CNVSTPin, LOW);

  SPI.begin();
  
  // ------------------------------
  Serial.begin(9600);
  delay(100);

  // Patent pending and copyright notice displayed at startup
  Serial.println( versionstr );
  Serial.println( authorstr );

  Serial.print("// F_CPU: "); Serial.print(F_CPU/1e6);  Serial.println(" MHz.");
#ifdef F_BUS
  Serial.print("// F_BUS: "); Serial.print(F_BUS/1e6);  Serial.println(" MHz.");
#endif

#ifdef ADC_F_BUS
  Serial.print("// ADC_F_BUS: "); Serial.print(ADC_F_BUS/1e6); Serial.println(" MHz.");
#endif

  measureoverhead();

#ifdef HASLED
  blink();
  delay(1000);
  blink();
  delay(1000);
#endif

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
      _rebootFunc();
    }

    /* --------------------------------------------------------------
     */
    else if (testKey(rcvbuffer, "help",0)) {
      Serial.println( "/* -----------------------------------------------");
      Serial.println( versionstr );
      Serial.println( authorstr );
      Serial.println("  reboot         - reboots the entire board");
#ifdef HAS_CPU_TEMPERATURE    
      Serial.println("  temperature    - report microcontroller temperature");
#endif
      Serial.println("  help           - display this message");
      Serial.println("  adc [pin]      - time analogRead");
      Serial.println("  set [npin] hi|low|input|pullup|output");
      Serial.println("  read [npin]    - time digital Read");
      Serial.println("  write [npin]   - time digital Write");
      Serial.println("  toggle [npin]  - time digital Toggle");
      Serial.println("  pulse [npin]   - time write high then low");
      Serial.println("  latency [npin_in npin_out] - time isr latency");
      Serial.println("  spi            - time spi transfers");
      Serial.println("  faux spi       - time manual loop spi transfers");
      Serial.println("  timer usecs    - time timer start and intervals");
#ifdef IS_TEENSY
      Serial.println(" Teensy specific fucntions");
      Serial.println("  macro          - time read macro");
      Serial.println("  fast adc [pin] - time analogReadFast");
      Serial.println("  fast latency [npin_in npin_out] - time isr latency");
      Serial.println("  fast spi       - time optimized spi readout transfers");
      Serial.println("");
#endif
      Serial.println("*/");
    }

#ifdef HAS_CPU_TEMPERATURE    
    else if (testKey(rcvbuffer,"temp",0)) {    
	Serial.print( "CHIPTEMPERATURE " );
	Serial.println( getCpuTemperature() );
      }
#endif

#ifdef IS_TEENSY
    else if (testKey( rcvbuffer, "fast adc", &pc)) {
       
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
	serial_printf( "//fastAnalogRead pin %d result %d\n", pin, u16tmp);
	serial_printf( "#define FAST_ADC_CYCLES %d\n", cpuavg);
	serial_printf( "#define FAST_ADC_NANOSECS %.1f\n", ((double)cpuavg/F_CPU)*1.E9);
	serial_printf( "#define FAST_ADC_CYCLES_MAX %d\n", cpumax);
	serial_printf( "#define FAST_ADC_NANOSECS_MAX %.1f\n", ((double)cpumax/F_CPU)*1.E9);
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
    
    else if (testKey( rcvbuffer, "fast latency", &pc)) {


      if (testUint(pc,&inputpin,&pc)&&testUint(pc,&outputpin,NULL)) {

	pinMode(inputpin,INPUT);
	pinMode(outputpin,OUTPUT);
	DIGITALWRITE( outputpin, LOW );

	directAttach(digitalPinToInterrupt(inputpin), timing_direct_isr, RISING);
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS; n++ ) {

	  cpucycles = 0;
	  elapsed_cycles_start();
	  DIGITALWRITE( outputpin, HIGH );

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
	DIGITALWRITE( OUTPIN, LOW );

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
	  DIGITALWRITE(OUTPIN,HIGH);

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
    
    else if (testKey( rcvbuffer, "fast spi", &pc)) {
      uint16_t saved_framesz;
      uint16_t data[NKNTS];
      float ravg = 0.;

      cpuavg = 0;
      cpumax = 0;
      
      //SPI.begin();
      SPI.beginTransaction(spi_settings);
      saved_framesz = get_framesz( lpspi );
      set_framesz( lpspi, 16 );

      for (uint16_t i=0; i<NKNTS; i++){ 

	DIGITALWRITE(CNVSTPin,HIGH);
	DELAYNANOSECONDS( 700 );
	DIGITALWRITE(CNVSTPin,LOW);

	elapsed_cycles_start();

	DIGITALWRITE(SPAREPIN,HIGH);
	data[i] = transfer16( lpspi, 0xFFFF );
	DIGITALWRITE(SPAREPIN,LOW);

	cpucycles = elapsed_cycles();
	cpuavg += cpucycles;
	if ( cpucycles > cpumax) cpumax = cpucycles;
      }

      set_framesz( lpspi, saved_framesz ); // restore original
      SPI.endTransaction();

      for (uint16_t i=0, ravg = 0; i<NKNTS; i++){
        ravg += data[i];
      }
      ravg /= NKNTS;
      serial_printf( "// average reading (binary) %.2f/n", ravg);

      sendResults( "SPI_fast_transfer16", cpuavg, cpumax, NKNTS);
    }

#endif

    else if (testKey( rcvbuffer, "set", &pc)) {
       
      if (testUint(pc,&pin,&pc)) {

        if (testKey(pc,"hi",NULL)) {
          serial_printf( "// Set pin %d high\n",pin);
	  DIGITALWRITE(pin,HIGH);
        }
        
        else if (testKey(pc,"low",NULL)) {
          serial_printf( "// Set pin %d low\n",pin);
	  DIGITALWRITE(pin,LOW);
        }
        
        else if (testKey(pc,"pullup",NULL)) {
          serial_printf( "// Set pin %d pullup\n",pin);
          pinMode(pin,INPUT_PULLUP);
        }
        else if (testKey(pc,"input",NULL)) {
          serial_printf( "// Set pin %d input\n",pin);
          pinMode(pin,INPUT);
        }
        else if (testKey(pc,"output",NULL)) {
          serial_printf( "// Set pin %d output\n",pin);
          pinMode(pin,OUTPUT);
        }
        else {
          Serial.println( "Error: set command, pin state not recognized");
        }
      }
      else {
        Serial.println( "Error: set command, pin number not recognized");
      }
    }
        
    else if (testKey( rcvbuffer, "cyccnt", &pc)) {

      uint32_t wait_cycles;
      uint32_t stop_cycles;
      uint32_t start_cycles;
      
      unsigned int nsecs = 0;

      Serial.println("Cyccnt" );

      if (testUint(pc,&nsecs,&pc) && testUint(pc,&pin,&pc)) {        

        wait_cycles = (((nsecs)*(F_CPU / 1000000))/1000);
        
        Serial.print("Cyccnt test: nsecs " );
        Serial.print( nsecs);
        Serial.print(" pin " );
        Serial.print( pin);
        Serial.print(" cpu clock " );
        Serial.print( F_CPU);
        Serial.print(" cycles wait " );
        Serial.println( wait_cycles);

        //Serial.println("high");
        digitalWrite(pin,HIGH);
        //DIGITALWRITE(pin,HIGH);

        start_cycles = ARM_DWT_CYCCNT;
        DELAYNANOSECONDS(5000);
        stop_cycles = ARM_DWT_CYCCNT;
        
        //Serial.println("low");
        digitalWrite(pin,LOW);
        //DIGITALWRITE(pin,LOW);
          
        Serial.print("  Result: start " );
        Serial.print( start_cycles);
        Serial.print("  until " );
        Serial.print( stop_cycles);
        Serial.print("  net " );
        Serial.println( stop_cycles-start_cycles);
      }
      else {
        Serial.println("Error cyccnt nsecs pin - missing nsecs pin");
      }

    }

    else if (testKey( rcvbuffer, "adc", &pc)) {
       
      if (testUint(pc,&pin,NULL)) {
	cpucycles = 0;
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS; n++ ) {
	  elapsed_cycles_start();
	  u16tmp = analogRead( pin );
	  cpucycles = elapsed_cycles();
	  cpuavg += cpucycles;
	  if ( cpucycles > cpumax) cpumax = cpucycles;
	}
	cpuavg /= NKNTS;
	cpuavg -= cpucycles_overhead;
	cpumax -= cpucycles_overhead;
	serial_printf( "//fastAnalogRead pin %d result %d\n", pin, u16tmp);
	serial_printf( "#define ANALOGREAD_CYCLES %d\n", cpuavg);
	serial_printf( "#define ANALOGREAD_NANOSECS %.1f\n", ((double)cpuavg/F_CPU)*1.E9);
	serial_printf( "#define ANALOGREAD_CYCLES_MAX %d\n", cpumax);
	serial_printf( "#define ANALOGREAD_NANOSECS_MAX %.1f\n", ((double)cpumax/F_CPU)*1.E9);
      }
      else {
	cpucycles = 0;
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS; n++ ) {
	  elapsed_cycles_start();
	  u16tmp = analogRead( ADCPIN );
	  cpucycles = elapsed_cycles();
	  cpuavg += cpucycles;
	  if ( cpucycles > cpumax) cpumax = cpucycles;
	}
	sendResults( "ANALOGREAD", cpuavg, cpumax, NKNTS);
      }
    }
    
    else if (testKey( rcvbuffer, "read", &pc)) {
       
      if (testUint(pc,&pin,NULL)) {
	cpucycles = 0;
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS; n++ ) {
	  elapsed_cycles_start();
	  u16tmp = DIGITALREAD( pin );
	  cpucycles = elapsed_cycles();
	  cpuavg += cpucycles;
	  if ( cpucycles > cpumax) cpumax = cpucycles;
	}
	sendResults( "DIGITAL_READ", cpuavg, cpumax, NKNTS);
      }
      else {
	cpucycles = 0;
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS; n++ ) {
	  elapsed_cycles_start();
	  u16tmp = DIGITALREAD( INPIN );
	  cpucycles = elapsed_cycles();
	  cpuavg += cpucycles;
	  if ( cpucycles > cpumax) cpumax = cpucycles;
	}
	sendResults( "DIGITAL_READ_CONST", cpuavg, cpumax, NKNTS);
      }
    }

    else if (testKey( rcvbuffer, "write", &pc)) {
      uint32_t ucycles1 = 0;
      uint32_t uavg1 = 0;
      uint32_t umax1 = 0;
      
      uint32_t ucycles2 = 0;
      uint32_t uavg2 = 0;
      uint32_t umax2 = 0;
      
      if (testUint(pc,&pin,NULL)) {
	for (int n = 0; n < NKNTS/2; n++ ) {

          ucycles1 = ARM_DWT_CYCCNT;
	  DIGITALWRITE( pin, HIGH );
          ucycles1 = ARM_DWT_CYCCNT - ucycles1;
          
          uavg1 += ucycles1;
          if (ucycles1 > umax1) umax1 = ucycles1;

	  DELAYNANOSECONDS(5000);
          	  
          ucycles2 = ARM_DWT_CYCCNT;
	  DIGITALWRITE( pin, LOW );
          ucycles2 = ARM_DWT_CYCCNT - ucycles2;

          uavg2 += ucycles2;
          if (ucycles2 > umax2) umax2 = ucycles2;

	  DELAYNANOSECONDS(5000);
        }
	sendResults( "DIGITAL_WRITE", (uavg1+uavg2)/2, (umax1>umax2?umax1:umax2), NKNTS);
	sendResults( "DIGITAL_WRITE_HIGH", uavg1, umax1, NKNTS);
	sendResults( "DIGITAL_WRITE_LOW", uavg2, umax2, NKNTS);
      }
      else {
	cpucycles = 0;
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS/2; n++ ) {
	  elapsed_cycles_start();
	  DIGITALWRITE( OUTPIN, HIGH );
	  cpucycles = elapsed_cycles();
	  cpuavg += cpucycles;
	  if ( cpucycles > cpumax) cpumax = cpucycles;

	  DELAYNANOSECONDS(5000);
	  
	  elapsed_cycles_start();
	  DIGITALWRITE( OUTPIN, LOW );
	  cpucycles = elapsed_cycles();
	  cpuavg += cpucycles;
	  if ( cpucycles > cpumax) cpumax = cpucycles;
	}
	sendResults( "DIGITAL_WRITE_CONST", cpuavg, cpumax, NKNTS);
      }
    }

    else if (testKey( rcvbuffer, "pulse", &pc)) {
       
      if (testUint(pc,&pin,NULL)) {
	cpucycles = 0;
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS; n++ ) {
	  elapsed_cycles_start();
	  DIGITALWRITE( pin, HIGH );
	  DIGITALWRITE( pin, LOW );
	  cpucycles = elapsed_cycles();
	  cpuavg += cpucycles;
	  if ( cpucycles > cpumax) cpumax = cpucycles;
	}
	sendResults( "DIGITAL_PULSE", cpuavg, cpumax, NKNTS);
      }
      else {
	cpucycles = 0;
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS; n++ ) {
	  elapsed_cycles_start();
	  DIGITALWRITE( OUTPIN, HIGH );
	  DIGITALWRITE( OUTPIN, LOW );
	  cpucycles = elapsed_cycles();
	  cpuavg += cpucycles;
	  if ( cpucycles > cpumax) cpumax = cpucycles;
	}
	sendResults( "DIGITAL_PULSE_CONST", cpuavg, cpumax, NKNTS);
      }
    }

    else if (testKey( rcvbuffer, "toggle", &pc)) {
       
      if (testUint(pc,&pin,NULL)) {
	cpucycles = 0;
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS; n++ ) {

	  elapsed_cycles_start();
	  DIGITALTOGGLE( pin );
	  cpucycles = elapsed_cycles();
	  cpuavg += cpucycles;
	  if ( cpucycles > cpumax) cpumax = cpucycles;

	}
	sendResults( "DIGITAL_TOGGLE", cpuavg, cpumax, NKNTS);
      }
      else {
	cpucycles = 0;
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS; n++ ) {

	  elapsed_cycles_start();
	  DIGITALTOGGLE( OUTPIN );
	  cpucycles = elapsed_cycles();
	  cpuavg += cpucycles;
	  if ( cpucycles > cpumax) cpumax = cpucycles;

	}
	sendResults( "DIGITAL_TOGGLE_CONST", cpuavg, cpumax, NKNTS);
      }
    }


    else if (testKey( rcvbuffer, "latency", &pc)) {


      if (testUint(pc,&inputpin,&pc)&&testUint(pc,&outputpin,NULL)) {

	pinMode(inputpin,INPUT);
	pinMode(outputpin,OUTPUT);
	DIGITALWRITE( outputpin, LOW );

	attachInterrupt(digitalPinToInterrupt(inputpin), timing_isr, RISING);

	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS; n++ ) {

	  cpucycles = 0;
	  elapsed_cycles_start();
	  DIGITALWRITE( outputpin, HIGH );

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
	DIGITALWRITE( OUTPIN, LOW );

	attachInterrupt(digitalPinToInterrupt(INPIN), timing_const_isr, RISING);
	
	cpuavg = 0;
	cpumax = 0;
	for (int n = 0; n < NKNTS; n++ ) {
	  
	  cpucycles = 0;
	  elapsed_cycles_start();
	  DIGITALWRITE(OUTPIN,HIGH);

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


    else if (testKey( rcvbuffer, "spi", &pc)) {

      uint16_t data[NKNTS];
      float ravg = 0.;

      //SPI.begin();
      SPI.beginTransaction(spi_settings);

      cpuavg = 0;
      cpumax = 0;
      for (int n = 0; n < NKNTS; n++ ) {

	DIGITALWRITE(CNVSTPin,HIGH);
	DELAYNANOSECONDS(700);
	DIGITALWRITE(CNVSTPin,LOW);
	
	elapsed_cycles_start();
	
	DIGITALWRITE(SPAREPIN,HIGH);
	data[n] = SPI.transfer16(0xFFFF);
	DIGITALWRITE(SPAREPIN,LOW);

	cpucycles = elapsed_cycles();
	cpuavg += cpucycles;
	if ( cpucycles > cpumax) cpumax = cpucycles;

      }

      for (uint16_t i=0, ravg = 0; i<NKNTS; i++){
        ravg += data[i];
      }
      ravg /= NKNTS;
      serial_printf( "// average reading (binary) %.2f/n", ravg);
      
      sendResults( "SPI_transfer16", cpuavg, cpumax, NKNTS);
	      
      SPI.endTransaction();
      //SPI.end();

    }
  
    else if (testKey( rcvbuffer, "faux spi", &pc)) {
      //uint32_t cyclesnext, cyclesnow;
      int d[32] = {0};
      int dummy = 0;
      
      pinMode(CNVSTPin,OUTPUT);
      pinMode(SDIPin,INPUT);
      pinMode(SDOPin,OUTPUT);
      pinMode(CLKPin,OUTPUT);

      DIGITALWRITE(CNVSTPin,LOW);
      DIGITALWRITE(CLKPin,LOW);
      DELAYNANOSECONDS(1000);
      
      DIGITALWRITE(CNVSTPin,HIGH);
      DELAYNANOSECONDS(700);
      DIGITALWRITE(CNVSTPin,LOW);
      
      DIGITALWRITE(SDOPin,HIGH);
      //cyclesnext = ARM_DWT_CYCCNT + 10;
      for (int n = 0; n < 16; n++ ) {

	/*
	while( (cyclesnow=ARM_DWT_CYCCNT) < cyclesnext);
	cyclesnext = cyclesnow + 10;
	*/
	DIGITALWRITE(CLKPin,HIGH);

	d[n] = DIGITALREAD(SDIPin);

	/*
	while( (cyclesnow=ARM_DWT_CYCCNT) < cyclesnext);
	cyclesnext = cyclesnow + 10;
	*/
	DIGITALWRITE(CLKPin,LOW);
	dummy = DIGITALREAD(SDIPin);

      }
      Serial.print("Result ");
      for (int n = 0; n < 16; n++ ) {
	Serial.print( d[n]&1 );
      }
      Serial.print(" ");
      Serial.print( dummy );

      dummy = 0;
      for (int n = 1; n < 16; n++ ) {
        dummy |= (d[n]<<n);
      }
      Serial.print(" ");
      Serial.print( dummy, HEX );
      Serial.print(" ");
      Serial.print( dummy );

      Serial.println("");
    }

    else if (testKey( rcvbuffer, "timer1", &pc)) {
      unsigned int usecs = 1000;
      unsigned int msecs;
      
      if (testUint(pc,&usecs,NULL)||usecs) {

        Serial.print( "// Timer1 period ");
        Serial.println( usecs);

        timerstart(usecs);

        msecs = (usecs * 10)/1000.;
        msecs = msecs > 100 ? msecs : 100;
        delay(msecs+100);

        serial_printf( "// Timer1 cycles %d %d time %f %f counter %d\n",
                       cpucycles, cpucycles2,
                       ((double)cpucycles/F_CPU)*1.E9,
                       ((double)cpucycles2/F_CPU)*1.E9,
                       timertest_countdown);

      }

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
