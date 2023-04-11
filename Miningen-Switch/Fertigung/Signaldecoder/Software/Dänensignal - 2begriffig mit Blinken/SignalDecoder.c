//------------------------------------------------------------------------
//
// OpenDCC - OpenDecoder - SignalDecoder
//
// Copyright (c) 2006-2010 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
// 
//
//  ATTENTION: USE WINAVR 20070122 - code wont fit with actual gcc!
//
//------------------------------------------------------------------------
//
// file:      SignalDecoder.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// webpage:   http://www.opendcc.de
// history:   2006-07-26 V0.2  copied from OpenDecoder
//                             added PWM Engine to run smooth transistions
//            2006-11-06 V0.3  single slope version copied to backup,
//                             working on versatile control engine.
//            2006-11-06 V0.4  now combined with rest-ontime-offtime-Engine
//                             from OpenDecoder.
//                             7 different modes implemented
//            2007-01-09 V0.5  added prog-key on-off toggle function
//                             removed traffic light and running light
//                             (see EffectDecoder)
//            2007-01-09 V0.6  traffic light added again
//            2007-05-31 V0.8  new mode 6 for Alex Shepherd, 8 turnout adresses
//            2007-10-24 V0.9  double decoding fixed.
//            2007-10-29 V0.10 new mode for belgian signals
//                             (with compile switch)
//            2008-02-03 V0.11 Interrupt sei() for belgian signals was omitted.
//                             changed preamble from 11 to 10
//            2010-02-22 V0.12 added Dutch Signals (compile switch)
//
//------------------------------------------------------------------------
//
// purpose:   flexible, lowcost decoder for dcc
//            use as decoder for signals with dimming engine
//
// content:   A DCC-Decoder for ATtiny2313 and other AVR
//
//             1. Defines and variable definitions
//             2. DCC receive routine
//             3. Timing values for the timing engine
//             4. Timing engine (pulse durations, blinking)
//             5. MAIN: analyze command, call the action, do programming
//             6. Test and Simulation
//
// issues:    up to now - only linear dimming
//            can't use gcc's srand() and rand() functions - approx. 400 bytes
//            of code! prbs-code from LightDecoder
//            
//
//------------------------------------------------------------------------
//
//
//
// DIESER DECODER VERWENDET DEN DUTCH-MODUS UND STEUERT SO JE 2 FARBEN (ROT/GRUEN) MIT BLINKEN. 
// MARTIN FITZEL 2020-06-15
//
//
//




#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>
#include <avr/pgmspace.h>        // put var to program memory
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <string.h>

#define SIMULATION  0            // 0: real application
                                 // 1: test receive routine
                                 // 2: test timing engine
                                 // 3: test action of running light

#define PRBS        0

#define BELGIAN_SIGNALS    1     // 0: standard mode
                                 // 1: only Belgian signals

#define DUTCH_SIGNALS      0     // 0: standard mode
                                 // 1: only Dutch signals

 


//---------------------------------------------------------------------
// Timing Definitions:
//
#ifndef F_CPU
   // prevent compiler error by supplying a default 
   # warning "F_CPU not defined for <SignalDecoder.c>", set default to 10MHz 
   # define F_CPU 10000000UL
   // if changed: check every place where it is used
   // (possible range underflow/overflow in preprocessor!)
#endif

#if (F_CPU == 10000000UL)
    unsigned char speed[] PROGMEM = {"..10MHz.."};
#endif
#if (F_CPU == 8000000UL)
    unsigned char speed[] PROGMEM = {"..8MHz.."};
#endif

/// This are DCC timing definitions from NMRA
#define PERIOD_1   116L          // 116us for DCC 1 pulse - do not change
#define PERIOD_0   232L          // 232us for DCC 0 pulse - do not change


// Definitions for the DIMM-Engine
// see below on information about the DIMM-Engine

#define PWM_TICK_PERIOD     300L        // 300us tick for PWM Engine
#define PWM_STEPS           60L
#define PWM_PERIOD          (PWM_TICK_PERIOD * PWM_STEPS)    // 18ms = 300 * 60L
#define DIMM_UP_DELAY       400000L     // Dimm up with 400ms delay 
#define DIMM_DOWN_SPEED     4L          // Dimm down slow 
#define DIMM_UP_SPEED       5L          // Dimm up a little faster
#define DIMM_DOWN_SAME_BULB 1           // 1: wenn im neuen Begriff die lampe 
                                        //    wieder leuchtet, zuerst abdimmen,
                                        //    dann wieder aufdimmen.
                                        // 0: leuchtet durchgehend  



//---------------------------------------------------------------------------
// PORT Definitions:
//
// PORTD:
#define PROGTASTER    0     // input, internal pullup
#define DCCIN         2     // must be located on INT0
#define JUMPER        4     // if fitted: save state to EEPROM
#define LED           6     // out

#define PROG_PRESSED   (!(PIND & (1<<PROGTASTER)))
#define JUMPER_FITTED  (!(PIND & (1<<JUMPER)))
#define LED_OFF        PORTD &= ~(1<<LED)
#define LED_ON         PORTD |= (1<<LED)

//----------------------------------------------------------------------------
// Global Data

// unsigned int  MyAdr;               // my DCC address
unsigned char MyOpMode;            // mode of operation

volatile unsigned char MyDelay;    // is decremented by ISR

unsigned int  ReceivedAdr;         // last received
unsigned char ReceivedCommand;


// 
unsigned char hauptsignal_state;    // für Dunkeltastung
unsigned char vorsignal_state;

//-----------------------------------------------------------------------------
// data in EEPROM:
//
unsigned char EE_empty     EEMEM = 0xff; // keep free
unsigned char EE_myAdrL    EEMEM = 0x05; // Decoder Adresse low
unsigned char EE_myAdrH    EEMEM = 0;    // Decoder Adresse high
unsigned char EE_myOpMode  EEMEM = 0;    // Pulse oder Dauer
unsigned char EE_LastState EEMEM = 0;    // aktueller Portzustand

//------------------------------------------------------------------------
// OpMode: (loaded by coil during programming)
// 0: Vier 2-begriffige Signale
// 1: Zwei 3-begriffige Signale
// 2: Einfahrsignal + Vorsignal getrennt 
// 3: Einfahrsignal + Vorsignal mit Dunkeltastung
// 4: zwei Wechselblinker (mit Ansteuerung Stopmagnet)
// 5: tbd. (?? ev. zwei Signale + 2 Schalter)
// 6: reserved 
// 7: Ampel mit Ansteuerung Stopmagnet
//
//---------------------------------------------------------------------------------
// Some tricks to optmize size and speed:
//
// 1. Save global flags (like Communicate or dcc.state) in unused IOs
// 2. Speed up port access by declaring port access as inline code
// 3. Cast logical operation down to char, whenever possible.
//    use assign to local variable and compare this var - smaller size
// 4. Same code in different cases should be identically ordered -
//    gcc reuses this code!
//
// Trick 1: Using IO as variables - what are unused IOs on ATtiny2313:
//   GPIOR0, GPIOR1, GPIOR2 General Purpose I/O Register 
//   UBRRL Baudrate, (USIDR USI Data Register)
//
     #define OPTIMIZE_VARS
//
//   We use:
//      GPIOR0 for general communication between processes
//      GPIOR1 for DCC ISR
//      GPIOR2 for Output State: 1 = ramp towards 1, 0 = ramp towards 0
    
     #ifdef OPTIMIZE_VARS
         #define Communicate    GPIOR0
         #define Recstate       GPIOR1
         #define CurrentTarget  GPIOR2   
     #elif
         unsigned char Communicate;
         unsigned char Recstate;
         unsigned char CurrentTarget;
                 
     #endif


// Trick 2: Speed up port access by declaring port access as inline code
//          This is done with a subroutine; if gcc runs with -Os, this results in
//          single cbi and sbi statements!

static inline void output(unsigned char port, unsigned char state) 
       __attribute__((always_inline));

void output(unsigned char port, unsigned char state)
  {
    if (state == 0)
      {                              
        PORTB &= (~(1<<port));
      }
    else
      {
        PORTB |= (1<<port);
      }
  }
                      
static inline void set_target(unsigned char port, unsigned char state) 
       __attribute__((always_inline));

void set_target(unsigned char port, unsigned char state)
  {
    if (state == 0)
      {                              
        CurrentTarget &= (~(1<<port));
      }
    else
      {
        CurrentTarget |= (1<<port);
      }
  }

#define WINAVR2007  1


#if (WINAVR2007 == 1)
#define my_eeprom_write_byte   eeprom_write_byte
#define my_eeprom_read_byte  eeprom_read_byte
#else

// wrapper functions for avoiding dumb inlining by gcc
void my_eeprom_write_byte(uint8_t *__p, uint8_t __value) __attribute__((noinline));
void my_eeprom_write_byte(uint8_t *__p, uint8_t __value)
  {
    eeprom_write_byte(__p, __value);
  }


uint8_t my_eeprom_read_byte(const uint8_t *__p) __attribute__((noinline));
uint8_t my_eeprom_read_byte(const uint8_t *__p)
  {
    return(eeprom_read_byte(__p));
  }                     

#endif
//----------------------------------------------------------------------------------
// Definitions for inter process communication


#define C_Received        0    // a new DCC was received - issued by ISR(Timer1)
                               //                          cleared by main
#define C_DoSave          1    // a new PORT state should be saved
                               //                        - issued by action
                               //                          cleared by main
#define C_Dimmstep        2    // perform recalculation of dimm_val
                               //                        - issued by do_dimm every 10ms
                               //                          cleared by main (dimmer)
    




void init_main(void)
  {
    DDRB  = 0xFF;               // PortB: All Bits as Output
    DDRD  = 0xFF                // PortD: all output but:
          & ~(1<<PROGTASTER)    // in - set to 0
          & ~(1<<DCCIN)  
          & ~(1<<JUMPER);
    PORTD =  (1<<PROGTASTER)    // pullup - set to 1
          |  (1<<DCCIN) 
          |  (1<<JUMPER);

    // Init Timer1

    TCCR1B = (0 << ICNC1) 
           | (0 << ICES1) 
           | (0 << WGM13) 
           | (1 << WGM12)       // Mode 4: CTC with OCRA1
           | (0 << CS12)        // clk stopped
           | (0 << CS11) 
           | (0 << CS10);

    // set Timer1 Compare to 3/4 of period of a one -> this is 116*0,75=87us
    
    OCR1A = F_CPU * PERIOD_1 * 3 / 4 / 1000000L;  



    // Init Timer0 as CTC 
    // check PWM_TICK_PERIOD and F_CPU
    #define TIMER0_CLKDIV 64              // possible values: 1, 8, 64, 256, 1024

    #if (F_CPU / 1000000L * PWM_TICK_PERIOD / TIMER0_CLKDIV) > 255L
      #warning: overflow in OCR0A - check TICK_PERIOD and F_CPU
      #warning: suggestion: use a larger clkdiv
    #endif    
    #if (F_CPU / 1000000L * PWM_TICK_PERIOD / TIMER0_CLKDIV) < 30L
      #warning: resolution accuracy in OCR0A too low - check TICK_PERIOD and F_CPU
      #warning: suggestion: use a smaller clkdiv
    #endif    


    OCR0A = F_CPU / 1000000L * PWM_TICK_PERIOD / TIMER0_CLKDIV ;  
    TCCR0A = (0 << COM0A1)      // compare match A
           | (0 << COM0A0) 
           | (0 << COM0B1)      // compare match B
           | (0 << COM0B0) 
           | 0                  // reserved
           | 0                  // reserved
           | (1 << WGM01)  
           | (0 << WGM00);      // Timer0 Mode 2 = CTC - Int on compare A
    TCCR0B = (0 << FOC0A) 
           | (0 << FOC0B) 
           | (0 << WGM02) 
    #if   TIMER0_CLKDIV == 1
           | (0 << CS02) | (0 << CS01)  | (1 << CS00);       // CS[2:0]=001 clkdiv 1
    #elif TIMER0_CLKDIV == 8
           | (0 << CS02) | (1 << CS01)  | (0 << CS00);       // CS[2:0]=010 clkdiv 8
    #elif TIMER0_CLKDIV == 64
           | (0 << CS02) | (1 << CS01)  | (1 << CS00);       // CS[2:0]=011 clkdiv 64 
    #elif TIMER0_CLKDIV == 256
           | (1 << CS02) | (0 << CS01)  | (0 << CS00);       // CS[2:0]=100 clkdiv 256
    #elif TIMER0_CLKDIV == 1024
           | (1 << CS02) | (0 << CS01)  | (1 << CS00);       // CS[2:0]=101 clkdiv 1024   
    #else
     #warning: TIMER0_CLKDIV is void
    #endif


    TIMSK = (0<<TOIE1)       // Timer1 Overflow
          | (1<<OCIE1A)      // Timer1 Compare A
          | (0<<OCIE1B)      // Timer1 Compare B
          | 0                // reserved
          | (0<<ICIE1)       // Timer1 Input Capture
          | (0<<OCIE0B)      // Timer0 Compare B
          | (0<<TOIE0)       // Timer0 Overflow
          | (1<<OCIE0A);     // Timer0 Compare A


    // Init Interrupt 0

    GIMSK = (0<<INT1)        // Int1 is msb
          | (1<<INT0)        // Enable INT0
          | (0<<PCIE); 


    MCUCR = 0x03;  //           ;The rising edge of INT0 generates an interrupt request.
          
  }






//==============================================================================
//
// Section 2
//
// DCC Receive Routine
//
// Howto:    uses two interrupt: a rising edge in DCC polarity triggers INT0;
//           in INT0, Timer1 with a delay of 87us is started.
//           On Timer1 Compare Match the level of DCC is evaluated and
//           parsed.
//
//                           |<-----116us----->|
//
//           DCC 1: _________XXXXXXXXX_________XXXXXXXXX_________
//                           ^-INT0
//                           |----87us--->|
//                                        ^-INT1: reads zero
//
//           DCC 0: _________XXXXXXXXXXXXXXXXXX__________________
//                           ^-INT0
//                           |----------->|
//                                        ^-INT1: reads one
//           
// Result:   1. The received message is stored in "message" and "message_size"
//           2. The flag C_Received is set.
//
#define MAX_MESSAGE 6    // including XOR-Byte
volatile unsigned char message[MAX_MESSAGE];
volatile unsigned char message_size;

struct
    {
        unsigned char state;                    // current state
        unsigned char bitcount;                 // current bit
        unsigned char bytecount;                // current byte
        unsigned char accubyte;                 // actual check
    } dccrec;

// some states:
#define RECSTAT_WF_PREAMBLE  0
#define RECSTAT_WF_LEAD0     1
#define RECSTAT_WF_BYTE      2
#define RECSTAT_WF_TRAILER   3

#define RECSTAT_DCC          7   


// ISR(INT0) loads only a register and stores this register to IO.
// this influences no status flags in SREG.
// therefore we define a naked version of the ISR with
// no compiler overhead.

#define ISR_INT0_OPTIMIZED

#ifdef ISR_INT0_OPTIMIZED
    #define ISR_MY_NAKED(vector) \
        void vector (void) __attribute__ ((signal, naked)); \
        void vector (void)

    ISR_MY_NAKED(INT0_vect) 
      {
         __asm__ __volatile 
          (
            "push r16"  "\n\t"
            "ldi r16, %1"  "\n\t"
            "out %0, r16" "\n\t"
            "pop r16"  "\n\t"
         :                         // no output section
         : "M" (_SFR_IO_ADDR (TCCR1B)),
           "M" ((0 << ICNC1)       // start timer1
              | (0 << ICES1) 
              | (0 << WGM13) 
              | (1 << WGM12)       // Mode 4: CTC
              | (0 << CS12)        // clk 1:1
              | (0 << CS11) 
              | (1 << CS10))
          );
        asm volatile ( "reti" ); 
      }
#else
    ISR(INT0_vect) 
      {
        TCCR1B = (0 << ICNC1)       // start timer1
               | (0 << ICES1) 
               | (0 << WGM13) 
               | (1 << WGM12)       // Mode 4: CTC
               | (0 << CS12)        // clk 1:1
               | (0 << CS11) 
               | (1 << CS10); 
      }

#endif

unsigned char copy[] PROGMEM = {"..SignalDecoder V0.12 .."};

ISR(TIMER1_COMPA_vect)
  {
    #define mydcc (Recstate & (1<<RECSTAT_DCC))

    // read asap to keep timing!
    if (PIND & (1<<DCCIN)) Recstate &= ~(1<<RECSTAT_DCC);  // if high -> mydcc=0
    else                   Recstate |= 1<<RECSTAT_DCC;    

    //sei(); !!!

    TCCR1B = (0 << ICNC1)       
           | (0 << ICES1) 
           | (0 << WGM13) 
           | (1 << WGM12) 
           | (0 << CS12)        // clk stopped
           | (0 << CS11) 
           | (0 << CS10);

    TCNT1 = 0;                  // clear Counter

    dccrec.bitcount++;

    if (Recstate & (1<<RECSTAT_WF_PREAMBLE))            // wait for preamble
      {                                       
        if (mydcc)
          {
            if (dccrec.bitcount >= 10) 
              {
                Recstate = 1<<RECSTAT_WF_LEAD0;            
              }
          }
        else
          {
            dccrec.bitcount=0;
          }
      }
    else if (Recstate & (1<<RECSTAT_WF_LEAD0))          // wait for leading 0
      {
        if (mydcc)
          {                                             // still 1, wait again
          }
        else
          {
            dccrec.bytecount=0;
            Recstate = 1<<RECSTAT_WF_BYTE;
            dccrec.bitcount=0;
            dccrec.accubyte=0;
          }
      }
    else if (Recstate & (1<<RECSTAT_WF_BYTE))           // wait for byte
      {
        unsigned char my_accubyte;
        my_accubyte = dccrec.accubyte << 1;
        if (mydcc)
          {
            my_accubyte |= 1;
          }
        dccrec.accubyte = my_accubyte;
        
        /* dccrec.accubyte = dccrec.accubyte << 1;
        if (mydcc)
          {
            dccrec.accubyte |= 1;
          }
         */
        if (dccrec.bitcount==8)
          {
            if (dccrec.bytecount == MAX_MESSAGE)        // too many bytes
              {                                         // ignore message
                Recstate = 1<<RECSTAT_WF_PREAMBLE;   
              }
            else
              {
                message[dccrec.bytecount++] = dccrec.accubyte;
                Recstate = 1<<RECSTAT_WF_TRAILER; 
              }
          }
      }
    else if (Recstate & (1<<RECSTAT_WF_TRAILER))        // wait for 0 (next byte) 
      {                                                 // or 1 (eof message)
        if (mydcc)
          {  // trailing "1" received
            Recstate = 1<<RECSTAT_WF_PREAMBLE;
            dccrec.bitcount=1;
            message_size = dccrec.bytecount;
            Communicate |= (1<<C_Received);
          }
        else
          {
            Recstate = 1<<RECSTAT_WF_BYTE;
            dccrec.bitcount=0;
            dccrec.accubyte=0;
          }
      }
    else
      {
        Recstate = 1<<RECSTAT_WF_PREAMBLE;
      }
  }

#if (SIMULATION == 1)

unsigned char dccbit;
void simulat_receive(void);

void dcc_receive(void)
  {
    #define mydcc dccbit

    dccrec.bitcount++;

    if (Recstate & (1<<RECSTAT_WF_PREAMBLE))            // wait for preamble
      {                                       
        if (mydcc)
          {
            if (dccrec.bitcount >= 10) 
              {
                Recstate = 1<<RECSTAT_WF_LEAD0;            
              }
          }
        else
          {
            dccrec.bitcount=0;
          }
      }
    else if (Recstate & (1<<RECSTAT_WF_LEAD0))          // wait for leading 0
      {
        if (mydcc)
          {                                             // still 1, wait again
          }
        else
          {
            dccrec.bytecount=0;
            Recstate = 1<<RECSTAT_WF_BYTE;
            dccrec.bitcount=0;
            dccrec.accubyte=0;
          }
      }
    else if (Recstate & (1<<RECSTAT_WF_BYTE))           // wait for byte
      {
        dccrec.accubyte = dccrec.accubyte << 1;
        if (mydcc)
          {
            dccrec.accubyte |= 1;
          }
        if (dccrec.bitcount==8)
          {
            if (dccrec.bytecount == MAX_MESSAGE)        // too many bytes
              {                                         // ignore message
                Recstate = 1<<RECSTAT_WF_PREAMBLE;   
              }
            else
              {
                message[dccrec.bytecount++] = dccrec.accubyte;
                Recstate = 1<<RECSTAT_WF_TRAILER; 
              }
          }
      }
    else if (Recstate & (1<<RECSTAT_WF_TRAILER))        // wait for 0 (next byte) 
      {                                                 // or 1 (eof message)
        if (mydcc)
          {  // trailing "1" received
            Recstate = 1<<RECSTAT_WF_PREAMBLE;
            dccrec.bitcount=1;
            message_size = dccrec.bytecount;
            Communicate |= (1<<C_Received);
          }
        else
          {
            Recstate = 1<<RECSTAT_WF_BYTE;
            dccrec.bitcount=0;
            dccrec.accubyte=0;
          }
      }
    else
      {
        Recstate = 1<<RECSTAT_WF_PREAMBLE;
      }
  }

#endif //  (SIMULATION == 1)


#define DIMM_ENGINE
#ifdef DIMM_ENGINE

//==============================================================================
//
// Section 3
//
// Lichtsteuerung als PWM-Dimmer
//
//             |                        |                          |
// Output:     |XXXXXXXXXX______________|XXXXXXXXXX________________|
//             |<-------->              |<------------------------>|
//             | dimm_val               |        PWM_PERIOD        |
//         ----|------------------------|--------------------------|---> Time
//
//
// 1. Einstellen des aktuellen Helligkeitswertes (do_dimm)
//
//    Es gibt 60 Helligkeitstufen.
//    Alle 300us erfolgt ein Interrupt, dieser schaltet den Dimmer um eine
//    Stufe weiter, nach 60 Stufen wird wieder von vorne begonnen.
//    Stufe MIN:  alle Ports mit einem dimm_val > DIMM_MIN werden eingeschaltet.
//    Stufe x:    Ein Port, dessen dimm_val kleiner x ist, wird abgeschaltet.
//    Stufe MAX:  Restart und Meldung an den DIMMER (C_Dimmstep)
//
//    Folge: Alle Ports mit einem dimm_val kleiner DIMM_RANGE_MIN sind
//    dauerhaft aus, alle Ports mit einem dimm_val größer DIMM_RANGE_MAX
//    sind dauerhaft ein.
//
//    Da der PWM 60 Stufen hat und alle 300us ein Int erfolgt, wird dieser
//    Durchlauf alle 18ms durchgeführt. Dies entspricht einer Refreshrate von
//    55Hz.
//
//
// 2. Langsame Veränderung der Helligkeit
//
//    Nach einem Zyklus des PWM wird vom Hauptprogramm der neue aktuelle
//    dimm_val ausgerechnet. Hierzu wird vom aktuellen Wert mit einem
//    Schritt "delta" nach oben oder unten gerechnet, bis der neue Zielwert
//    erreicht ist.
//
//    Die Zykluszeit der PWM ist 18ms, somit wird je nach "delta" folgende
//    Dimmzeit erreicht:
//
//      delta    |    Dimmzeit
//    -----------------------------
//        1      |    1080ms
//        2      |     540ms
//        3      |     360ms
//        4      |     270ms
//        5      |     216ms
//        6      |     180ms
//      100      |    sofort
//
//    Der neue Zielwert wird in light_val hinterlegt. 
//
//    Wenn man von einem Wert kleiner DIMM_RANGE_MIN startet, dann wird
//    der Port erst mit Verzögerung aufgedimmt, weil zuerst der Bereich
//    bis DIMM_RANGE_MIN "aufgedimmt" wird.
//    Dies wird dazu benutzt, zuerst das alte Signalbild wegzudimmen 
//    und dann das neue Signalbild aufzudimmen.
//
// 3. Signalbilder
// 
//    Signalbilder werden als Bitfeld hinterlegt. Für jedes Kommando
//    gibt es ein Bitfeld, in dem das neue Signalbild abgelegt ist und eine
//    Gültigkeitsmaske, diese bestimmt, auf welche Dimmwerte das Signalbild
//    wirken soll. Mit diesen beiden Pattern wird "set_new_light_val" 
//    aufgerufen.
//
// 4. Blinken
//    
//    Falls ontime bzw. offtime ungleich 0 sind, wird nach Ablauf der
//    jeweils andere Phasenwert geladen.
//

#define DIMM_RANGE_MIN 100    // aktiver Bereich 100-160
#define DIMM_RANGE_MAX (DIMM_RANGE_MIN+PWM_STEPS+1)



//------------------------------------------- Array for DIMM-Engine
// a) running values
unsigned char cur_dimm;            //  = DIMM_RANGE_MIN;


// b) target values
volatile struct
  {
    unsigned char rest;     	// Zeit bis zum nächsten Wechsel in PWM_PERIOD (18ms)
    unsigned char ontime;   	// Einschaltzeit
    unsigned char offtime;      // Ausschaltzeit
    unsigned char dimm_val;     // aktueller Istwert
    unsigned char light_A_val;  // aktueller Zielwert in Richtung A
    unsigned char delta_A;      // aktuelles Increment in Richtung A
    unsigned char light_B_val;  // aktueller Zielwert in Richtung B
    unsigned char delta_B;
  } out_pwm[8];

//
// void do_dimm(void)
ISR(TIMER0_COMPA_vect)                      // Timer0 Compare Int
  {                                         // macht pwm
    unsigned char port;
    unsigned char mask;

    sei();
    mask = 1;
    cur_dimm++;
    if (cur_dimm == DIMM_RANGE_MAX)
      {
        cur_dimm = DIMM_RANGE_MIN;
        for (port=0; port<8; port++)
          {
            if (out_pwm[port].dimm_val > DIMM_RANGE_MIN) PORTB |= mask;   // Einschalten wenn !0
            mask = mask << 1; 
          }
        Communicate |= (1<<C_Dimmstep);
        if (MyDelay) MyDelay--; 
      }
    else
      {
        for (port=0; port<8; port++)
          {
            if (cur_dimm >= out_pwm[port].dimm_val) PORTB &= ~mask; 
            mask = mask << 1;
          }
      }
  }


// Diese Funktion setzt die neuen Ziel-Bits für DIMM
// beim Aufdimmen gibt es immer 400ms Verzögerung, damit ein Abdimmen vorher fertig wird.


void set_new_light_val(unsigned char pattern, unsigned char valid)
  {
    unsigned char port;
    unsigned char mask;
    unsigned char inv_pattern = ~pattern;

    mask = 1;
    for (port=0; port<8; port++)
      {
        if (pattern & mask & valid)
          {
            // turn on
            #if DIMM_DOWN_SAME_BULB == 1
                out_pwm[port].rest = DIMM_UP_DELAY / PWM_PERIOD;   
                CurrentTarget &= ~mask;
            #else
                if (CurrentTarget & mask)
                  {
                    // already on, nothing to do, keep burning 
                  }
                else
                  {
                    out_pwm[port].rest = DIMM_UP_DELAY / PWM_PERIOD;   
                    CurrentTarget &= ~mask;
                  }
            #endif
          }
        if (inv_pattern & mask & valid)
          {
            // turn off
            out_pwm[port].rest    = 0;
            CurrentTarget &= ~mask;
          }
        mask = mask << 1;
      }
  }




//---------------------------------------------------------------------------------
// dimmer()
// this routine is called from main(), if C_Dimmstep is activated  
//

// Timing Engine
//
// Howto:    
// 1. Generelles Timing:
//    Diese Routine wird alle PWM_PERIOD aufgerufen. Es wird folgendes
//    geprüft:
//    a) Wenn out_pwm[port].rest gleich 0: dann bleibt dieser Port unverändert.
//    b) out_pwm[port].rest wird decrementiert, wenn es dabei 0 wird, dann
//       wird ein Dimmvorgang in die andere Richtung eingeleitet.
//
// 2. Dimm-Übergänge:
//    Je nach aktueller Richtung des Dimmvorgang (CurrentTarget) wird der aktuelle
//    Dimmwert erhöht oder erniedrigt (z.Z. linear).
//    Die Dimmrampe ist unabhängig von den Zeiten, die bei ontime bzw. offtime
//    vorgegeben werden.
//    Wenn ein Ausgang schalten soll, dann muß sein Delta sehr groß gewählt
//    werden!
  


void dimmer(void)
  {
    unsigned char port;
    unsigned char mask;
    unsigned char my_rest;

    mask = 1;
    for (port=0; port<8; port++)
      {
        my_rest = out_pwm[port].rest;       // use a local variable to force
        if (my_rest !=0)                    // compiler to tiny code
          { 
            if (--my_rest == 0)
              {
                 if (CurrentTarget & mask)
                  { // bit was on
                    my_rest = out_pwm[port].offtime;
                    CurrentTarget &= ~mask;
                  }
                else
                  {
                    my_rest = out_pwm[port].ontime;
                    CurrentTarget |= mask;
                  }
              }
            out_pwm[port].rest = my_rest;
          }


    	my_rest = out_pwm[port].dimm_val;
        if (CurrentTarget & mask)    
          {                                     // we are in phase A -> incr
            if (my_rest < out_pwm[port].light_A_val)  
              {
                // fehlt Sicherung gegen Überlauf -> bei Daten aufpassen
                // delta darf nicht zu groß sein
				my_rest += out_pwm[port].delta_A;
              }
          }
        else
          {                                     // we are in phase B -> decr
            if (my_rest > out_pwm[port].light_B_val)
              {
                if (my_rest > out_pwm[port].delta_B)
					 my_rest -= out_pwm[port].delta_B;
                else my_rest = 0;
              }
          }
        out_pwm[port].dimm_val = my_rest;
        mask = mask << 1;                  // do *not* calc mask from port
      }
  }


#endif // DIMM_ENGINE
//==============================================================================
//

//==============================================================================
//
// Section 4
//
// Pattern for different signals
//
// a) Vierfach rot grün
//
// Anschluß:
//
//  0baabbccdd     Anschluß:
//    |  ||  |---- Hauptsignal rot
//    |  || |----- Hauptsignal grün

unsigned char EE_Signal_DB2_Pattern[8] EEMEM =
  {
    0b00000001,          // hp0
    0b00000010,          // hp1
    0b00000100,
    0b00001000,
    0b00010000,
    0b00100000,
    0b01000000,
    0b10000000,
  };

unsigned char EE_Signal_DB2_Valid[8] EEMEM =
  {
    0b00000011,
    0b00000011,
    0b00001100,
    0b00001100,
    0b00110000,
    0b00110000,
    0b11000000,
    0b11000000,
  };


// b)  zwei unabhängige mehrbegriffige Vorsignale / Hauptsignale (a/b)
//
// Anschluß:
//
//  0baaaabbbb     Anschluß für:
//    |  ||  |---- Vorsignal gelb 1;   Hauptsignal rot
//    |  || |----- Vorsignal gelb 2;   unused
//    |  |||------ Vorsignal grün 1;   Hauptsignal grün
//    |  ||------- Vorsignal grün 2;   Hauptsignal gelb

unsigned char EE_Signal_DB3_Pattern[8] EEMEM =
  {
    0b00000011,     // vr0         hp0
    0b00001100,     // vr1         hp2
    0b00000110,     // vr2         hp1
    0b00000000,     // unused      unused
    0b00110000,     // vr0         hp0
    0b11000000,     // vr1         hp2
    0b01100000,     // vr2         hp1
    0b00000000,     // unused      unused
  };

unsigned char EE_Signal_DB3_Valid[8] EEMEM =
  {
    0b00001111,
    0b00001111,
    0b00001111,
    0b00001111,
    0b11110000,
    0b11110000,
    0b11110000,
    0b11110000,
  };


//
// Hauptsignal und Vorsignal am gleichen Mast (mit Dunkeltastung)
//
// Anschluß:
//    0bvvvhhhhh
//      | ||   |---- HS rot 1
//      | ||  |----- HS rot 2
//      | || |------ HS gelb
//      | |||------- HS grün
//      | ||-------- HS weiß
//      | |--------- VS vr0 (Dioden erforderlich, siehe Opendecoder)
//      ||---------- VS vr1
//      |----------- VS vr2

unsigned char EE_Signal_DB4_Pattern[8] EEMEM =
  {
    0b00000011,     // hp0
    0b00001000,     // hp1
    0b00001100,     // hp2
    0b00010001,     // sh1
    0b00100000,     // vr0
    0b01000000,     // vr1
    0b10000000,     // vr2
    0b00000000,     // Dunkeltastung
  };

unsigned char EE_Signal_DB4_Valid[8] EEMEM =
  {
    0b00011111,
    0b00011111,
    0b00011111,
    0b00011111,
    0b11100000,
    0b11100000,
    0b11100000,
    0b11100000,
  };


//
// Alex Shepherd - 4 aspects binary coded (red, green, both on, none)
// Mode 6:

unsigned char EE_Signal_comb_Pattern[16] EEMEM =
  {
    0b00000001,    // only red    
    0b00000010,    // only green 
    0b00000011,    // red and green
    0b00000000,    // dark 
    0b00000100,     
    0b00001000,     
    0b00001100,     
    0b00000000,  
    0b00010000,    // only red    
    0b00100000,    // only green 
    0b00110000,    // red and green
    0b00000000,    // dark 
    0b01000000,     
    0b10000000,     
    0b11000000,     
    0b00000000,     
  };

unsigned char EE_Signal_comb_Valid[16] EEMEM =
  {
    0b00000011,
    0b00000011,
    0b00000011,
    0b00000011,
    0b00001100,
    0b00001100,
    0b00001100,
    0b00001100,
    0b00110000,
    0b00110000,
    0b00110000,
    0b00110000,
    0b11000000,
    0b11000000,
    0b11000000,
    0b11000000,
  };


unsigned char EE_Signal_bin_Pattern[16] EEMEM =
  {
    0b00000001,    // only red    
    0b00000000,    // red off
    0b00000010,    // green
    0b00000000,    // off
    0b00000100,     
    0b00000000,     
    0b00001000,     
    0b00000000,  
    0b00010000,    // red on   
    0b00000000,    // off
    0b00100000,    // green on
    0b00000000,    // off
    0b01000000,     
    0b00000000,     
    0b10000000,     
    0b00000000,     
  };

unsigned char EE_Signal_bin_Valid[16] EEMEM =
  {
    0b00000001,
    0b00000001,
    0b00000010,
    0b00000010,
    0b00000100,
    0b00000100,
    0b00001000,
    0b00001000,
    0b00010000,
    0b00010000,
    0b00100000,
    0b00100000,
    0b01000000,
    0b01000000,
    0b10000000,
    0b10000000,
  };


//==============================================================================
//
// Section 5
//
// MAIN: analyze command, call the action, do programming
//

//------------------------------------------------------------------------------
// This Routine is called when myAdr is received

#if (BELGIAN_SIGNALS == 1)
#warning BELGIAN_SIGNAL enabled, different decoding!

// UMBAU ALS DAENISCHES SIGNAL FUER 2 BEGRIFFIGE SIGNALE MIT BLINKEN VON GRUEN!  MARTIN FITZEL 


void action(void)
  {

	// Daenische Signal
    // there are 8 lights and ? aspects. So we can two signals, using 16 addresses
    // map
    // > > addr +0, r: single red                 1. Signal
    // > > addr +0, g: single green
    // > > addr +1, r: single green flashing
    // > > addr +1, g: -
    // > > addr +2, r: single red 				  2. Signal
    // > > addr +2, g: single green
    // > > addr +3, r: single green flashing
    // > > addr +3, g: -
    // > > addr +4, r: single red                 3. Signal
    // > > addr +4, g: single green
    // > > addr +5, r: single green flashing
    // > > addr +5, g: -
    // > > addr +6, r: single red                 4. Signal
    // > > addr +6, g: single green
    // > > addr +7, r: single green flashing
    // > > addr +7, g: -
	
	

    // connect as follows:
    // Port 0  4: green
    // Port 1  5: red
    // Port 2  6: yellow (in row)
    // Port 3  7: yellow (aside)

    #define BELG_FLASH  ( 900000L / PWM_PERIOD)   // Period
    
    unsigned char myCommand;
    unsigned char i;
    
    myCommand = ReceivedCommand & 0b00001111;

    cli();                      // block interrupts
    
    Communicate |= (1<<C_DoSave); 

    if (myCommand < 8)                                      // clear any pending flashing
      {
        for (i=0;i<4;i++)
              {
                out_pwm[i].offtime = 0;  
                out_pwm[i].ontime = 0;  
              }       
      }
    else
      {
        for (i=4;i<8;i++)
              {
                out_pwm[i].offtime = 0;  
                out_pwm[i].ontime = 0;  
              }       
      }

    
    switch(myCommand)
      {
        case 0:                                             // single red 
            set_new_light_val(0b00000010, 0b00000011); 
            break;
        case 1:                                             // single green
            set_new_light_val(0b00000001, 0b00000011); 
            break;
        case 2:                                             // single green flashing
            out_pwm[0].offtime = BELG_FLASH/2;  
            out_pwm[0].ontime = BELG_FLASH/2;  
            set_new_light_val(0b00000101, 0b00000011); 
            break;
		case 3:                                     		// all off (neu: gruen)
            set_new_light_val(0b00000001, 0b00000011);
            break; 
        case 4:                                             // single red 
            set_new_light_val(0b00001010, 0b00001100); 
            break;
        case 5:                                             // single green
            set_new_light_val(0b00000101, 0b00001100); 
            break;
        case 6:                                             // single green flashing
            out_pwm[2].offtime = BELG_FLASH/2;  
            out_pwm[2].ontime = BELG_FLASH/2;  
            set_new_light_val(0b00000100, 0b00001100); 
            break;
        case 7:                                             // all off
            set_new_light_val(0b00000000, 0b00001100);
            break;
		case 8:                                             // single red 
            set_new_light_val(0b00100010, 0b00110000); 
            break;
        case 9:                                             // single green
            set_new_light_val(0b00010001, 0b00110000); 
            break;
        case 10:                                             // single green flashing
            out_pwm[4].offtime = BELG_FLASH/2;  
            out_pwm[4].ontime = BELG_FLASH/2;  
            set_new_light_val(0b00010001, 0b00110000); 
            break;
        case 11:                                             // all off
            set_new_light_val(0b00000000, 0b00110000);
            break;
		case 12:                                             // single red 
            set_new_light_val(0b10000010, 0b11000000); 
            break;
        case 13:                                             // single green
            set_new_light_val(0b01000001, 0b11000000); 
            break;
        case 14:                                             // single green flashing
            out_pwm[6].offtime = BELG_FLASH/2;  
            out_pwm[6].ontime = BELG_FLASH/2;  
            set_new_light_val(0b01000001, 0b11000000); 
            break;
        case 15:                                             // all off
            set_new_light_val(0b00000000, 0b11000000);
            break; 			
			
			
			
      }
    sei();
  }

#elif (DUTCH_SIGNALS == 1)   // Belgian_signal
#warning DUTCH_SIGNAL enabled, different decoding!

// contributed by Cees Baarda (c.baarda@hccnet.nl)
void action(void)
  {
    // signalling according to Dutch signals
    // there are 4 lights and 8 aspects. So we can do two signals, using 16 addresses
    // map
    // > > addr +0, r: single red                 1. Signal
    // > > addr +0, g: single green
    // > > addr +1, r: yellow and digit
    // > > addr +1, g: yellow
    // > > addr +2, r: green flashing and digit
    // > > addr +2, g: green flashing
    // > > addr +3, r: yellow and digit flashing
    // > > addr +3, g: yellow flashing
    // > > addr +4, r: single red                 2. Signal
    // > > addr +4, g: single green
    // > > addr +5, r: yellow and digit
    // > > addr +5, g: yellow
    // > > addr +6, r: green flashing and digit
    // > > addr +6, g: green flashing
    // > > addr +7, r: yellow and digit flashingg
    // > > addr +7, g: yellow flashing

    // connect as follows:
    // Port 0  4: red
    // Port 1  5: yellow
    // Port 2  6: green
    // Port 3  7: digit

    #define DUTCH_FLASH  ( 900000L / PWM_PERIOD)   // Period
    
    unsigned char myCommand;
    unsigned char i;
    
    myCommand = ReceivedCommand & 0b00001111;      // limit to 16 aspects

    cli();                      // block interrupts
    
    Communicate |= (1<<C_DoSave); 

    if (myCommand < 8)                                      // clear any pending flashing
      {
        for (i=0;i<4;i++)
              {
                out_pwm[i].offtime = 0;  
                out_pwm[i].ontime = 0;  
              }       
      }
    else
      {
        for (i=4;i<8;i++)
              {
                out_pwm[i].offtime = 0;  
                out_pwm[i].ontime = 0;  
              }       
      }

    switch(myCommand)
      {
        case 0:                                             // red 
            set_new_light_val(0b00000001, 0b00001111); 
            break;
        case 1:                                             // green
            set_new_light_val(0b00000100, 0b00001111); 
            break;
        case 2:                                             // yellow + digit
            set_new_light_val(0b00001010, 0b00001111); 
            break;
        case 3:                                             // yellow
            set_new_light_val(0b00000010, 0b00001111); 
            break;
        case 4:                                             // green flashing +digit
            out_pwm[2].offtime = DUTCH_FLASH/2;  
            out_pwm[2].ontime = DUTCH_FLASH/2;  
            set_new_light_val(0b00001100, 0b00001111);
            break;
        case 5:                                             // green flashing
            out_pwm[2].offtime = DUTCH_FLASH/2;  
            out_pwm[2].ontime = DUTCH_FLASH/2;  
            set_new_light_val(0b00000100, 0b00001111); 
            break;
        case 6:                                             // yellow + digit flashing
            out_pwm[3].offtime = DUTCH_FLASH/2;  
            out_pwm[3].ontime = DUTCH_FLASH/2;  
            set_new_light_val(0b00001010, 0b00001111); 
            break;
        case 7:                                             // yellow flashing
            out_pwm[1].offtime = DUTCH_FLASH/2;  
            out_pwm[1].ontime = DUTCH_FLASH/2;  
            set_new_light_val(0b00000010, 0b00001111);
            break; 

        case 8:                                             // red 
            set_new_light_val(0b00010001, 0b11110000); 
            break;
        case 9:                                             // green
            set_new_light_val(0b01000100, 0b11110000); 
            break;
        case 10:                                             // yellow + digit
            set_new_light_val(0b10101010, 0b11110000); 
            break;
        case 11:                                             // yellow
            set_new_light_val(0b00100010, 0b11110000); 
            break;
        case 12:                                             // green flashing +digit
            out_pwm[6].offtime = DUTCH_FLASH/2;  
            out_pwm[6].ontime = DUTCH_FLASH/2;  
            set_new_light_val(0b11001100, 0b11110000);
            break;
        case 13:                                             // green flashing
            out_pwm[6].offtime = DUTCH_FLASH/2;  
            out_pwm[6].ontime = DUTCH_FLASH/2;  
            set_new_light_val(0b01000100, 0b11110000); 
            break;
        case 14:                                             // yellow + digit flashing
            out_pwm[7].offtime = DUTCH_FLASH/2;  
            out_pwm[7].ontime = DUTCH_FLASH/2;  
            set_new_light_val(0b10101010, 0b11110000); 
            break;
        case 15:                                             // yellow flashing
            out_pwm[5].offtime = DUTCH_FLASH/2;  
            out_pwm[5].ontime = DUTCH_FLASH/2;  
            set_new_light_val(0b00100010, 0b11110000);
            break; 
	      }
      sei();
  }

#else   // DUTCH_SIGNAL
// now standard mode

void action(void)
  {
    unsigned char myCommand;
    
    myCommand = ReceivedCommand & 0b00001111;

    cli();                      // block interrupts
    
    Communicate |= (1<<C_DoSave); 
        

    if (MyOpMode == 7)
      {
        // Traffic Light (Ampel)
        // Ampel
        // LED0:RT1 xxxxxxxx_____________________________xxxxxxxxxxxxxxxxx
        // LED1:GE1 ______xx___________________________xx_________________
        // LED2:GN1 ________xxxxxxxxxxxxxxxxxxxxxxxxxxx___________________
        // LED3:STOP1 xxxxxxx___________________________xxxxxxxxxxxxxxxxxx

        #define AMP_YE  (1000000L / PWM_PERIOD)   // Yellow 1 s
        #define AMP_BL  ( 500000L / PWM_PERIOD)   // Blinken
         if (myCommand == 0) // = schalte auf rot
           {

                  set_target(3,0); // Magnet
                  set_target(2,0); // gn;
                  set_target(1,1); // ye;
                  set_target(0,0); // rd;

                  out_pwm[0].rest    = AMP_YE;
                  // out_pwm[0].ontime  = 0;

                  out_pwm[1].rest    = AMP_YE;
                  out_pwm[1].offtime = 0;

                  out_pwm[2].rest    = 0;

                  out_pwm[3].rest    = AMP_YE / 2;  // StopMagnet nach halber Gelb ein
                  // out_pwm[3].ontime = 0;
                  // Hinweis: braucht großes Delta - wird beim Einstellen des Modes gemacht!

           }
         else if (myCommand == 1) // = schalte auf grün
           {
                  set_target(3,1); // Magnet
                  set_target(2,0); // gn;
                  set_target(1,1); // ye;
                  set_target(0,1); // rd;

                  out_pwm[0].rest    = AMP_YE;
                  //out_pwm[0].offtime  = 0;

                  out_pwm[1].rest    = AMP_YE;
                  out_pwm[1].offtime = 0;

                  out_pwm[2].rest    = AMP_YE;
                  //out_pwm[2].ontime = 0;

                  out_pwm[3].rest    = 3*AMP_YE / 2;   // Magnet nach 1,5 gelb aus
                  //out_pwm[3].offtime = 0;
           }
         else if (myCommand == 2) // = alles aus
           {
                  set_new_light_val(0x00, 0x0f);

                  /*
                  CurrentTarget = (0<<3) | (0<<2) | (0<<1) | (0<<0);
                                      //              Stop     green    gelb   rot
                  out_pwm[1].rest    = 0;
                  out_pwm[0].rest    = 0;
                  out_pwm[2].rest    = 0;
                  out_pwm[3].rest    = 0;
                  */
           }
         else if (myCommand == 3) // blinken
           {
                  set_target(3,0); // Magnet
                  set_target(2,0); // gn;
                  set_target(1,1); // ye;
                  set_target(0,0); // rd;

                  out_pwm[1].rest    = AMP_YE;
                  out_pwm[1].ontime  = AMP_YE;
                  out_pwm[1].offtime = AMP_YE;

                  out_pwm[0].rest    = 0;
                  out_pwm[2].rest    = 0;
                  out_pwm[3].rest    = 0;
           }
         else if (myCommand == 4) // = schalte auf rot
           {
                  set_target(7,0); // Magnet
                  set_target(6,0); // gn;
                  set_target(5,1); // ye;
                  set_target(4,0); // rd;

                  out_pwm[4].rest    = AMP_YE;
                  out_pwm[4].ontime  = 0;

                  out_pwm[5].rest    = AMP_YE;
                  out_pwm[5].offtime = 0;

                  out_pwm[6].rest    = 0;

                  out_pwm[7].rest    = AMP_YE / 2;
                  out_pwm[7].ontime = 0;
           }
         else if (myCommand == 5) // = schalte auf grün
           {
                  set_target(7,1); // Magnet
                  set_target(6,0); // gn;
                  set_target(5,1); // ye;
                  set_target(4,1); // rd;

                  out_pwm[4].rest    = AMP_YE;
                  out_pwm[4].offtime  = 0;

                  out_pwm[5].rest    = AMP_YE;
                  out_pwm[5].offtime = 0;

                  out_pwm[6].rest    = AMP_YE;
                  out_pwm[6].ontime = 0;

                  out_pwm[7].rest    = 3*AMP_YE / 2;
                  out_pwm[7].offtime = 0;

           }
         else if (myCommand == 6) // = alles aus
           {
                  set_new_light_val(0x00, 0xf0);

                  /*
                  CurrentTarget = (0<<7) | (0<<6) | (0<<5) | (0<<4);
                                      //              Stop     green    gelb   rot
                  out_pwm[5].rest    = 0;
                  out_pwm[4].rest    = 0;
                  out_pwm[6].rest    = 0;
                  out_pwm[7].rest    = 0;
                  */
           }
         else if (myCommand == 7) // blinken
           {
                  set_target(7,0); // Magnet
                  set_target(6,0); // gn;
                  set_target(5,1); // ye;
                  set_target(4,0); // rd;

                  out_pwm[5].rest    = AMP_YE;
                  out_pwm[5].ontime  = AMP_YE;
                  out_pwm[5].offtime = AMP_YE;

                  out_pwm[4].rest    = 0;
                  out_pwm[6].rest    = 0;
                  out_pwm[7].rest    = 0;
           }
      }
    else if (MyOpMode == 6)
      {                            // new for alex shepherd - uses all 16 commands
        set_new_light_val(
            my_eeprom_read_byte(&EE_Signal_comb_Pattern[myCommand]),
            my_eeprom_read_byte(&EE_Signal_comb_Valid[myCommand])    );
      }
    else if (MyOpMode == 5)
      {
        set_new_light_val(
            my_eeprom_read_byte(&EE_Signal_bin_Pattern[myCommand]),
            my_eeprom_read_byte(&EE_Signal_bin_Valid[myCommand])    );
      }
    else if (MyOpMode == 4)
      {                                            // Wechselblinker mit Stopmagnet
         // Port 0, 1: Andreaskreuze -> Wechselblinken 
         // Port 2: schnelles Blinkausgang (Lokführer)
         // Port 3: Schalten des Stopmagneten
         if (myCommand == 0) // = schalte alles ab
           {
             #define BUE_AK_BL  ( 1000000L / PWM_PERIOD)   // Blinken Andreaskreuz
             #define BUE_LF_BL  ( 1000000L / PWM_PERIOD)   // Blinken Lokführer
         
                  set_new_light_val(0x00, 0x07);          // turn all off
                  set_target(3,1); // Magnet;
                  out_pwm[3].rest    = 2* BUE_AK_BL;  // StopMagnet nach 2 Blinkphasen aus
                  out_pwm[3].offtime = 0;                   
				  // Hinweis: braucht großes Delta - wird beim Einstellen des Modes gemacht!
                  
           }
         else if (myCommand == 1) // = schalte beide ein, Phasenversatz
           {
                  set_target(3,0); // Magnet
                  set_target(2,0); // gn;
                  set_target(1,0); // ye;
                  set_target(0,0); // rd;
                  
                  out_pwm[0].rest    = 2*BUE_AK_BL/2;
                  out_pwm[0].offtime = BUE_AK_BL/2;
                  out_pwm[0].ontime  = BUE_AK_BL/2;
                
                  
                  out_pwm[1].rest    = BUE_AK_BL/2;
                  out_pwm[1].offtime = BUE_AK_BL/2;
                  out_pwm[1].ontime  = BUE_AK_BL/2;
                
                  out_pwm[2].rest    = BUE_LF_BL/2;
                  out_pwm[2].offtime = BUE_LF_BL/2;
                  out_pwm[2].ontime  = BUE_LF_BL/2;
                
                  out_pwm[3].rest    = 2* BUE_AK_BL;  // StopMagnet nach 2 Blinkphasen an
                  out_pwm[3].ontime = 0;                   
				  // Hinweis: braucht großes Delta - wird beim Einstellen des Modes gemacht!
           
           }
         else if (myCommand == 2) // unused
           {
          }                     
         else if (myCommand == 3) // unused
           {
           }
         else if (myCommand == 4) 
           {          
           // = schalte alles ab (2. Gruppe)
                  
                  set_new_light_val(0x00, 0x70);
                  set_target(7,1); // Magnet
                  out_pwm[7].rest    = 2* BUE_AK_BL;  // StopMagnet nach 2 Blinkphasen aus
                  out_pwm[7].offtime = 0;                   
				  // Hinweis: braucht großes Delta - wird beim Einstellen des Modes gemacht!
                  
           }
         else if (myCommand == 5) // = schalte beide ein, Phasenversatz
           {
                  set_target(7,0); // Magnet
                  set_target(6,0); // gn;
                  set_target(5,0); // ye;
                  set_target(4,0); // rd;
                  
                  out_pwm[4].rest    = 2*BUE_AK_BL/2;
                  out_pwm[4].offtime = BUE_AK_BL/2;
                  out_pwm[4].ontime  = BUE_AK_BL/2;
                
                  out_pwm[5].rest    = BUE_AK_BL/2;
                  out_pwm[5].offtime = BUE_AK_BL/2;
                  out_pwm[5].ontime  = BUE_AK_BL/2;
                
                  out_pwm[6].rest    = BUE_LF_BL/2;
                  out_pwm[6].offtime = BUE_LF_BL/2;
                  out_pwm[6].ontime  = BUE_LF_BL/2;
                
                  out_pwm[7].rest    = 2* BUE_AK_BL;  // StopMagnet nach 2 Blinkphasen
                  out_pwm[7].ontime = 0;                   
				  // Hinweis: braucht großes Delta - wird beim Einstellen des Modes gemacht!
           }
         else if (myCommand == 6) // unused
           {
                
           }                     
         else // (myCommand == 7) // unused
           {
                
           }
      }
    else if (MyOpMode == 3)
      {   											// 3: Einfahrsignal + Vorsignal mit Dunkeltastung
        if (myCommand <= 3)
          {
            if ((hauptsignal_state == 0)||(hauptsignal_state == 3)) 
              {
                // VS war aus, wieder anmachen
                set_new_light_val(
                    my_eeprom_read_byte(&EE_Signal_DB4_Pattern[vorsignal_state]),
                    my_eeprom_read_byte(&EE_Signal_DB4_Valid[vorsignal_state])    );  
              }
            hauptsignal_state = myCommand;
            set_new_light_val(
                my_eeprom_read_byte(&EE_Signal_DB4_Pattern[hauptsignal_state]),
                my_eeprom_read_byte(&EE_Signal_DB4_Valid[hauptsignal_state])    ); 
          }
        else if (myCommand <= 7)
          {
            vorsignal_state = myCommand;
            set_new_light_val(
                my_eeprom_read_byte(&EE_Signal_DB4_Pattern[vorsignal_state]),
                my_eeprom_read_byte(&EE_Signal_DB4_Valid[vorsignal_state])    );
          }

        if ((hauptsignal_state == 0)||(hauptsignal_state == 3)) 
          {                                        // mit Dunkeltastung
            // einfach drüber schreiben
            set_new_light_val(
                my_eeprom_read_byte(&EE_Signal_DB4_Pattern[7]),
                my_eeprom_read_byte(&EE_Signal_DB4_Valid[7])    );
          }
      }
    else if (MyOpMode == 2)         
      {                             // Einfahrsignal + Vorsignal getrennt
        if (myCommand < 8)
          {
            set_new_light_val(
                my_eeprom_read_byte(&EE_Signal_DB4_Pattern[myCommand]),
                my_eeprom_read_byte(&EE_Signal_DB4_Valid[myCommand])    );
          }
      }
    else if (MyOpMode == 1)        
      {                             // Zwei dreibegriffige Signale (DB)
        if (myCommand < 8)
          {
            set_new_light_val(
                my_eeprom_read_byte(&EE_Signal_DB3_Pattern[myCommand]),
                my_eeprom_read_byte(&EE_Signal_DB3_Valid[myCommand])    );
          }
      }
    else
      {   // MyOpMode == 0          // Vier zweibegriffige Signale
        if (myCommand < 8)
          {
            set_new_light_val(
                my_eeprom_read_byte(&EE_Signal_DB2_Pattern[myCommand]),
                my_eeprom_read_byte(&EE_Signal_DB2_Valid[myCommand])    );
          }
      }  // OpMode
    sei();
  }


#endif  // Belgian_signal
//
//-----------------------------------------------------------------------------
// analyze_message checks the received DCC message
// return 0 if void, 1 if accessory command, 2 if myAdr;
//
unsigned char analyze_message(void)
  {
    unsigned char i;
    unsigned char myxor = 0;
    unsigned int MyAdr;
    
    for (i=0; i<message_size; i++)
      {
        myxor = myxor ^ message[i];
      }

    if (myxor)
      {
        // checksum error, ignore
        return(0);
      }
    else
      {
        // check, if it is an accessory message (128-192)
        myxor = message[0] & 0b11000000;
        if (myxor == 0b10000000)
          {
            if (message[1] >= 0b10000000)  // MSB in Command byte set
              {
                if (message[1] & (1<<3))   // Bit 3: accessory command + active coil?
                  {
                    ReceivedCommand = message[1] & 0b00000111;
    
                    // take bits 5 4 3 2 1 0 from message[0]
                    // take Bits 6 5 4 from message[1] and invert
    
                    #define OPTCODE1
                    #ifdef OPTCODE1
                        unsigned char temp;
                        myxor = ~message[1] & 0b01110000;
                        myxor = myxor<<1;  // shift as byte
                        temp = message[0] & 0b00111111;
                        ReceivedAdr = (myxor<<1) | temp;
                    #else 
                        ReceivedAdr = (message[0] & 0b00111111)
                                    | ((~message[1] & 0b01110000) << 2);
                    #endif
    
                    MyAdr = (my_eeprom_read_byte(&EE_myAdrH) << 8) |
                            (my_eeprom_read_byte(&EE_myAdrL));
    
                    if (ReceivedAdr == MyAdr) return(2);
                    else if (ReceivedAdr == (MyAdr+1))
                      {
                        ReceivedCommand += 8;
                        return(2);
                      } 
                    else return(1);
                  }
              }
          }
      }
    return(0);
  }


//------------------------------------------------------------------------
// This Routine is called when PROG is pressed
//
#define DEBOUNCE  (50000L / PWM_PERIOD)
#if (DEBOUNCE == 0)
 #define DEBOUNCE  1
#endif


void DoProgramming(void)
  {
    unsigned char myCommand;    

    cli();
    MyDelay = DEBOUNCE;
    sei();

    while(MyDelay) ;     // wait until ISR has decremented  MyDelay

    if (PROG_PRESSED)                    // still pressed?
      {
        LED_ON;
        Communicate &= ~(1<<C_Received);
        
        while(PROG_PRESSED) ;           // wait for release

        cli();
        MyDelay = DEBOUNCE;
        sei();

        while(MyDelay) ;     // wait until ISR has decremented  MyDelay
        

        while(!PROG_PRESSED)
          {
            if (Communicate & (1<<C_Received))
              {                                     // Message
                Communicate &= ~(1<<C_Received);
                if (analyze_message())              // Accessory
                  {
                    my_eeprom_write_byte(&EE_myAdrL, (unsigned char) ReceivedAdr);     
                    my_eeprom_write_byte(&EE_myAdrH, (unsigned char) (ReceivedAdr >> 8));
                    
                    myCommand = ReceivedCommand & 0x07;
                    my_eeprom_write_byte(&EE_myOpMode, myCommand);
                    MyOpMode = myCommand;

                    my_eeprom_write_byte(&EE_LastState, 0);
                    
                    do {} while (!eeprom_is_ready());    // wait for write to complete
                    
                    LED_OFF;
                    
                    // we got reprogrammed ->
                    // forget everthing running and restart decoder!                    
                    
                    // cli();
                    
                    // laut diversen Internetseiten sollte folgender Code laufen -
                    // tuts aber nicht, wenn man das Assemblerlistung ansieht.
                    // void (*funcptr)( void ) = 0x0000;    // Set up function pointer
                    // funcptr();                        // Jump to Reset vector 0x0000
                    
                    __asm__ __volatile 
                    (
                       "ldi r30,0"  "\n\t"
                       "ldi r31,0"  "\n\t"
                       "icall" "\n\t"
                     );
                    // return;  
                  }
              }
          }  // while
        LED_OFF;
        cli();
        MyDelay = DEBOUNCE;
        sei();
        while(MyDelay) ;     // wait until ISR has decremented  MyDelay
        while(PROG_PRESSED) ;           // wait for release
      }
    return;   
  }


#if (PRBS_CODE == 1)
// linear feedback shift register (prbs)
//
//      |---|    |---|    |---|    |---|    |---|    |---|    |---|    |---|
//    ->| 0 |--->| 1 |-o->| 2 |-o->| 3 |-o->| 4 |--->| 5 |--->| 6 |--->| 7 |--o--->
//   |  |---|    |---| |  |---| |  |---| |  |---|    |---|    |---|    |---|  |
//   |                 |        |        |                                    |
//    <--------------- + <----- + <----- + <----------------------------------
//
unsigned char prbs8(unsigned char seed)
  {
    unsigned char new_rnd;

    new_rnd = seed;                // copy bit 1
    new_rnd = new_rnd << 1;
    new_rnd = new_rnd ^ seed;      // xor bit 2
    new_rnd = new_rnd << 1;
    new_rnd = new_rnd ^ seed;      // xor bit 3
    new_rnd = new_rnd << 4;
    new_rnd = new_rnd ^ seed;      // xor bit 7

    // now put this bit to seed's lsb
    new_rnd = new_rnd >> 7;
    seed = seed << 1;
    new_rnd = new_rnd + seed; 
    
    return(new_rnd);
  }
#endif


int main(void)
  {
    unsigned char port;
    init_main();
     

    
    
    // Delta für Glühlampensimulation vorbelegen

    for (port=0; port<8; port++)
          {
            out_pwm[port].dimm_val    = DIMM_RANGE_MIN;
            out_pwm[port].delta_A     = DIMM_UP_SPEED; 
            out_pwm[port].light_A_val = DIMM_RANGE_MAX;                
			out_pwm[port].delta_B     = DIMM_DOWN_SPEED;
			out_pwm[port].light_B_val = DIMM_RANGE_MIN;
                 
          }

   
    MyOpMode  = my_eeprom_read_byte(&EE_myOpMode);

    if ((MyOpMode == 4) || (MyOpMode == 7))   // Bahnübergang oder Ampel
	  {
            out_pwm[3].delta_A = PWM_STEPS+2; // Stopmagnet soll durchschalten
            out_pwm[3].delta_B = PWM_STEPS+2; 
 	        out_pwm[7].delta_A = PWM_STEPS+2; // Stopmagnet soll durchschalten
            out_pwm[7].delta_B = PWM_STEPS+2; 
 	  } 

    CurrentTarget = my_eeprom_read_byte(&EE_LastState);
    
    Communicate = 0; 
    Recstate = 1<<RECSTAT_WF_PREAMBLE;  

    sei();              // Global enable interrupts

    while(1)
      {
        cli();
        if (Communicate & (1<<C_Received)  )
          {
            sei();
            if (analyze_message() == 2)     // MyAdr empfangen
              {
                action();
              }
            Communicate &= ~(1<<C_Received); 
          }
        sei();
        sei();

        cli();
        if (Communicate & (1<<C_DoSave) )
          {
            sei();
            Communicate &= ~(1<<C_DoSave);
            if (JUMPER_FITTED)
              {
                my_eeprom_write_byte(&EE_LastState, CurrentTarget);   
              } 
          } 
        sei();
        sei();
        cli();
        if (Communicate & (1<<C_Dimmstep) )
          {
            sei();
            Communicate &= ~(1<<C_Dimmstep);
            dimmer();
          }

        if (PROG_PRESSED) DoProgramming();         
      }
  }


//=========================================================================
//
// Simulationen:
//
// 0: keine Simulation
// 1: Test der Empfangrootine; hierzu ein DCC Generator (aus opendcc/dccout.c)
// 2: Test der Portengine
//
#if (SIMULATION == 1)

//                                        Adressbyte   Datenbyte 
//                                         10AAAAAA    1aaaSCCR    // note: aaa = ~AAA
unsigned char message_adr001_out0_g[] = {0b10000001, 0b11111001};
unsigned char message_adr005_out1_g[] = {0b10000101, 0b11111011};
unsigned char message_adr380_out0_g[] = {0b10111100, 0b10101001};  // 380 = 0x17C = 0b101111100


// upstream interface:
struct
  {
    unsigned char size;
    unsigned char dcc[5];
  } next_message;

volatile unsigned char next_message_count;

enum do_states
  {                            // actual state
     dos_idle,
     dos_send_preamble,
     dos_send_bstart,
     dos_send_byte,
     dos_send_xor
  };

struct
  {
    enum do_states state;

    unsigned char ibyte;                            // current index of byte in message
    unsigned char bits_in_state;                    // Bits to output in this state
    unsigned char cur_byte;                         // current byte
    unsigned char xor_byte;                              // actual xor
    unsigned char current_dcc[5];                  // current message in output processing
    unsigned char bytes_in_message;                 // current size of message (decremented)
    unsigned char phase;
  } doi;


unsigned char dccbit;

void do_send(unsigned char mydccbit)
  {
    dccbit = mydccbit;
  }

void dcc_bit_generator(void)
  {
    switch (doi.state)
      {
        case dos_idle:
            do_send(1);
            if (next_message_count > 0)
              {
                memcpy(doi.current_dcc, next_message.dcc, sizeof(doi.current_dcc));
                doi.bytes_in_message = next_message.size;
                // no size checking - if (doi.cur_size > 5) doi.cur_size = 5;
                next_message_count--;
                doi.ibyte = 0;
                doi.xor_byte = 0;
                doi.bits_in_state = 14;
                doi.state = dos_send_preamble;
              }
            break;

        case dos_send_preamble:
            do_send(1);
            doi.bits_in_state--;
            if (doi.bits_in_state == 0)
                 doi.state = dos_send_bstart;
            break;

        case dos_send_bstart:
            do_send(0);
            if (doi.bytes_in_message == 0)
              { // message done, goto xor
                doi.cur_byte = doi.xor_byte;
                doi.state = dos_send_xor;
                doi.bits_in_state = 8;
              }
            else
              { // get next addr or data
                doi.bytes_in_message--;
                doi.cur_byte = doi.current_dcc[doi.ibyte++];
                doi.xor_byte ^= doi.cur_byte;
                doi.state = dos_send_byte;
                doi.bits_in_state = 8;
              }
            break;

        case dos_send_byte:
            if (doi.cur_byte & 0x80) do_send(1);
            else                    do_send(0);
            doi.cur_byte <<= 1;
            doi.bits_in_state--;
            if (doi.bits_in_state == 0)
              {
                doi.state = dos_send_bstart;
              }
            break;

        case dos_send_xor:
            if (doi.cur_byte & 0x80) do_send(1);
            else                    do_send(0);
            doi.cur_byte <<= 1;
            doi.bits_in_state--;
            if (doi.bits_in_state == 0)
              {
                doi.state = dos_idle;
              }
            break;
     }
  }

void dcc_generate_init(void)
  {
    doi.state = dos_idle;
  }


void simulat_receive(void)
  {
    dcc_generate_init();
    dcc_bit_generator();
    dcc_bit_generator();
    memcpy(next_message.dcc, message_adr001_out0_g, sizeof(doi.current_dcc));
    next_message.size = 2;
    next_message_count = 2;
    while(1)
      {
        dcc_bit_generator();
        dcc_receive();

        if (Communicate & (1<<C_Received)  )
          {
            if (analyze_message() == 2)     // MyAdr empfangen
              {
                action();
              }
            Communicate &= ~(1<<C_Received); 
          }
        if (Communicate & (1<<C_DoSave) )
          {
            Communicate &= ~(1<<C_DoSave);
            if (JUMPER_FITTED)
              {
                my_eeprom_write_byte(&EE_LastState, PORTB);   
              } 
          } 
        // if (PROG_PRESSED) DoProgramming();         
      }
 
  }


#endif // (SIMULATION == 1)


#if (SIMULATION == 2)
void simulat_mode_0_command_0(void)
{
  ReceivedCommand = 0x08; // Bit 3: active coil
  action();

      while(1)
      {
        sei();
        sei();
        cli();
        if (Communicate & (1<<C_Dimmstep) )
          {
            sei();
            Communicate &= ~(1<<C_Dimmstep);
            dimmer();
          }
      }

}


#endif // (SIMULATION == 2)

#if (SIMULATION == 3)
void simulat_mode_6_command_1(void)
{
  MyOpMode = 6;
  ReceivedCommand = 0b00001001; // Bit 3: active coil
  action();

      while(1)
      {
        sei();
        sei();
        cli();
        if (Communicate & (1<<C_Dimmstep) )
          {
            sei();
            Communicate &= ~(1<<C_Dimmstep);
            dimmer();
          }
      }

}


#endif // (SIMULATION == 3)
