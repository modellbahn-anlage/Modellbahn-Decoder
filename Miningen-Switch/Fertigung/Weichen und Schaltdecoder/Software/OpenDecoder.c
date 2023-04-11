//------------------------------------------------------------------------
//
// OpenDCC - OpenDecoder
//
// Copyright (c) 2006 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
// 
//------------------------------------------------------------------------
//
// file:      opendecoder.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// webpage:   http://www.opendcc.de
// history:   2006-06-13 V0.1 started
//            2006-07-11 V0.2 added some simulations;
//                            receive of DCC and timing engine simulated
//            2006-07-13 V0.3 direct loading of out_pwm
//                            test programming, test outputs done
//            2007-04-12 V0.4 Testversion - invertierte Ausgänge
//                            #define INVERT_OUTPUT
//                            dcc preamble bit reduced to 10.
//
//------------------------------------------------------------------------
//
// purpose:   flexible, lowcost decoder for dcc
//            use as pulse decoder or permanent decoder
//
// content:   A DCC-Decoder for ATtiny2313 and other AVR
//
//             1. Defines and variable definitions
//             2. DCC receive routine
//             3. Timing engine (pulse durations, blinking)
//             4. Timing values for the timing engine
//             5. MAIN: analyze command, call the action, do programming
//             6. Test and Simulation
//
//------------------------------------------------------------------------

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
                                 // 3: test action

#define INVERT_OUTPUT 0          // 0: standard
                                 // 1: IO inverted
                                 // Note: ontime, offtime changes as well!

//---------------------------------------------------------------------
// Timing Definitions:
//
#ifndef F_CPU
   // prevent compiler error by supplying a default 
   # warning "F_CPU not defined for <OpenDecoder.c>", set default to 10MHz 
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

#define TICK_PERIOD 25000L       // 25ms tick for Timing Engine
                                 // => possible values for timings up to
                                 //    6.3s (=255/0.025)


//---------------------------------------------------------------------------
// PORT Definitions:
//
// PORTD:
#define PROGTASTER    0
#define DCCIN         2     // must be located on INT0
#define JUMPER        4     // if Jumper fitted: state is saved in EEPROM
#define LED           6     // output

#define PROG_PRESSED   (!(PIND & (1<<PROGTASTER)))
#define JUMPER_FITTED  (!(PIND & (1<<JUMPER)))
#define LED_OFF        PORTD &= ~(1<<LED)
#define LED_ON         PORTD |= (1<<LED)

//----------------------------------------------------------------------------
// Global Data

// unsigned int  MyAdr;               // my DCC address
unsigned char MyOpMode;            // mode of operation
unsigned char pulsdelay;           // Delay for turnout

volatile unsigned char MyDelay;    // is decremented by ISR

unsigned int  ReceivedAdr;         // last received
unsigned char ReceivedCommand;


//-----------------------------------------------------------------------------
// data in EEPROM:
//
unsigned char EE_empty     EEMEM = 0xff; // keep free
unsigned char EE_myAdrL    EEMEM = 0x05; // Decoder Adresse low
unsigned char EE_myAdrH    EEMEM = 0;    // Decoder Adresse high
unsigned char EE_myOpMode  EEMEM = 3;    // Pulse oder Dauer
unsigned char EE_Pulsdelay EEMEM = 300000L / TICK_PERIOD;    // Pulsdauer
unsigned char EE_LastState EEMEM = 0;    // aktueller Portzustand

//------------------------------------------------------------------------
// OpMode:
// 0: Pulse 0,25s
// 1: Pulse 0,5s
// 2: Pulse 1s
// 3: Pulse 2s
// 4: Switch 2-states
// 5: Switch 3-states
// 6: Switch 4-states
// 7: Traffic Light (Ampel)
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
    
     #ifdef OPTIMIZE_VARS
         #define Communicate GPIOR0
         #define Recstate GPIOR1
     #elif
         unsigned char Communicate;
         unsigned char Recstate;         
     #endif


// Trick 2: Speed up port access by declaring port access as inline code
//          This is done with a subroutine; if gcc runs with -os, this results in
//          single cbi and sbi statements!

static inline void output(unsigned char port, unsigned char state) 
       __attribute__((always_inline));

void output(unsigned char port, unsigned char state)
  {
    if (state == 0)
      {                              
        #if (INVERT_OUTPUT == 1)
          PORTB |= (1<<port);
        #else
          PORTB &=  (~(1<<port));
        #endif 
      }
    else
      {
        #if (INVERT_OUTPUT == 1)
          PORTB &= (~(1<<port));
        #else
          PORTB |= (1<<port);
        #endif 
      }
  }
                      


//----------------------------------------------------------------------------------
// Definitions for inter process communication


#define C_Received        0    // a new DCC was received - issued by ISR(Timer1)
                               //                          cleared by main
#define C_DoSave          1    // a new PORT state should be saved
                               //                        - issued by action
                               //                          cleared by main
    




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


    // set Timer Compare to 3/4 of period of a one -> this is 116*0,75=87us
    
    OCR1A = F_CPU * PERIOD_1 * 3 / 4 / 1000000L;  



    // Init Timer0 as CTC 
    // check TICK_PERIOD and F_CPU

    #if (F_CPU / 1000000L * TICK_PERIOD / 1024) > 255L
      #warning: overflow in OCR0A - check TICK_PERIOD and F_CPU
      #warning: suggestion: use a larger clkdiv
    #endif    
    #if (F_CPU / 1000000L * TICK_PERIOD / 1024) < 50L
      #warning: resolution accuracy in OCR0A too low - check TICK_PERIOD and F_CPU
      #warning: suggestion: use a smaller clkdiv
    #endif    


    OCR0A = F_CPU / 1000000L * TICK_PERIOD / 1024 ;  
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
           | (1 << CS02)  
           | (0 << CS01)  
           | (1 << CS00);       // clkdiv 1024

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
    #define ISR_NAKED(vector) \
        void vector (void) __attribute__ ((signal, naked)); \
        void vector (void)

    ISR_NAKED(INT0_vect) 
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

unsigned char copy[] PROGMEM = {"..Opendecoder v0.4 (c) Kufer 2006,2008.."};

ISR(TIMER1_COMPA_vect)
  {
    #define mydcc (Recstate & (1<<RECSTAT_DCC))

    // read asap to keep timing!
    if (PIND & (1<<DCCIN)) Recstate &= ~(1<<RECSTAT_DCC);  // if high -> mydcc=0
    else                   Recstate |= 1<<RECSTAT_DCC;    

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
            if (dccrec.bitcount >= 10)                  // changed 06.02.2008 
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

#endif   // SIMULATION == 1



//==============================================================================
//
// Section 3
//
// Timing Engine
//
// Howto:    Timer0 (with prescaler 1024 and 8 bit total count) triggers
//           an interrupt every TICK_PERIOD (=26ms @10MHz);
//           this interrupt decrements out_pwm.rest of every port.
//           if out_pwm touches 0 a new value is reloaded.
//           This result in flexible programable timing of PORTB.
// 
  

volatile struct
  {
    unsigned char rest;     // Zeit bis zum wechsel in Ticks (5ms)
    unsigned char ontime;   // Einschaltzeit
    unsigned char offtime;  // Ausschaltzeit
  } out_pwm[8];


ISR(TIMER0_COMPA_vect)                      // Timer0 Compare Int
  {
    unsigned char port;
    unsigned char mask; 
    unsigned char my_rest; 

    sei();                                  // allow DCC interrupt
    if (MyDelay) MyDelay--;

    mask = 1;
    for (port=0; port<8; port++)
      {
        my_rest = out_pwm[port].rest;       // use a local variable to force
        if (my_rest !=0)                    // compiler to tiny code
          {
            if (--my_rest == 0)
              {
                 if (PORTB & mask)
                  { // bit was on
                    my_rest = out_pwm[port].offtime;
                    PORTB &= ~mask;
                  }
                else
                  {
                    my_rest = out_pwm[port].ontime;
                    PORTB |= mask;
                  }
              }
            out_pwm[port].rest = my_rest;
          }
         mask = mask << 1;                  // do *not* calc mask from port
      }
  }



//==============================================================================
//
// Section 4
//
// Timing Values for the Timing Engine
//
// All output control instructions are stored in tables. These tables are
// evaluated, when a action a pin is be performed.
//
// If a port is never toggled from the timing engine, it my be directly set.
//
struct t_action
  {
    unsigned char status;           // Multi purpose byte:
                                    // Bit 7:    new state of output pin
                                    // Bit 6:    variable first pulse enabled
                                    // Bit 2..0: output pin number
    unsigned char rest;             // time to keep the state; if 0: keep forever
    unsigned char ontime;           // ontime: if elapsed, output will turn off
    unsigned char offtime;          // 
  };


/*------------- Some predefined structures

struct t_action port0_on PROGMEM =   { (1 << 7) + 0, 0, 0, 0};
struct t_action port1_on PROGMEM =   { (1 << 7) + 1, 0, 0, 0};
struct t_action port2_on PROGMEM =   { (1 << 7) + 2, 0, 0, 0};
struct t_action port3_on PROGMEM =   { (1 << 7) + 3, 0, 0, 0};
struct t_action port4_on PROGMEM =   { (1 << 7) + 4, 0, 0, 0};
struct t_action port5_on PROGMEM =   { (1 << 7) + 5, 0, 0, 0};
struct t_action port6_on PROGMEM =   { (1 << 7) + 6, 0, 0, 0};
struct t_action port7_on PROGMEM =   { (1 << 7) + 7, 0, 0, 0};

struct t_action port0_off PROGMEM =  { (0 << 7) + 0, 0, 0, 0};
struct t_action port1_off PROGMEM =  { (0 << 7) + 1, 0, 0, 0};
struct t_action port2_off PROGMEM =  { (0 << 7) + 2, 0, 0, 0};
struct t_action port3_off PROGMEM =  { (0 << 7) + 3, 0, 0, 0};
struct t_action port4_off PROGMEM =  { (0 << 7) + 4, 0, 0, 0};
struct t_action port5_off PROGMEM =  { (0 << 7) + 5, 0, 0, 0};
struct t_action port6_off PROGMEM =  { (0 << 7) + 6, 0, 0, 0};
struct t_action port7_off PROGMEM =  { (0 << 7) + 7, 0, 0, 0};

//                                        "ON"     "PULS"  NR  
struct t_action port0_pulse PROGMEM =   { (1<<7) + (1<<6) + 0, 0, 0, 0};
struct t_action port1_pulse PROGMEM =   { (1<<7) + (1<<6) + 1, 0, 0, 0};
struct t_action port2_pulse PROGMEM =   { (1<<7) + (1<<6) + 2, 0, 0, 0};
struct t_action port3_pulse PROGMEM =   { (1<<7) + (1<<6) + 3, 0, 0, 0};
struct t_action port4_pulse PROGMEM =   { (1<<7) + (1<<6) + 4, 0, 0, 0};
struct t_action port5_pulse PROGMEM =   { (1<<7) + (1<<6) + 5, 0, 0, 0};
struct t_action port6_pulse PROGMEM =   { (1<<7) + (1<<6) + 6, 0, 0, 0};
struct t_action port7_pulse PROGMEM =   { (1<<7) + (1<<6) + 7, 0, 0, 0}; */



/*/                                           "ON" NR       rest,   ontime, offtime
struct t_action port0_amp1_rot    PROGMEM = { (0<<7) + 0,  AMP_YE, 0,      0   };
struct t_action port1_amp1_rot    PROGMEM = { (1<<7) + 1,  AMP_YE, 0,      0   };
struct t_action port3_amp1_rot    PROGMEM = { (0<<7) + 3,AMP_YE/2, 0,      0   };

struct t_action port0_amp1_green  PROGMEM = { (1<<7) + 0,  AMP_YE, 0,      0   };
struct t_action port1_amp1_green  PROGMEM = { (1<<7) + 1,  AMP_YE, 0,      0   };
struct t_action port2_amp1_green  PROGMEM = { (0<<7) + 2,  AMP_YE, 0,      0   };
struct t_action port3_amp1_green  PROGMEM = { (1<<7) + 3,3*AMP_YE/2, 0,      0   };

struct t_action port1_amp1_blink  PROGMEM = { (1<<7) + 1,  AMP_BL, AMP_BL, AMP_BL };

//
struct t_action port4_amp2_rot    PROGMEM = { (0<<7) + 4,  AMP_YE, 0,      0   };
struct t_action port5_amp2_rot    PROGMEM = { (1<<7) + 5,  AMP_YE, 0,      0   };
struct t_action port7_amp2_rot    PROGMEM = { (0<<7) + 7,AMP_YE/2, 0,      0   };

struct t_action port4_amp2_green  PROGMEM = { (1<<7) + 4,  AMP_YE, 0,      0   };
struct t_action port5_amp2_green  PROGMEM = { (1<<7) + 5,  AMP_YE, 0,      0   };
struct t_action port6_amp2_green  PROGMEM = { (0<<7) + 6,  AMP_YE, 0,      0   };
struct t_action port7_amp2_green  PROGMEM = { (1<<7) + 7,3*AMP_YE/2, 0,      0   };

struct t_action port5_amp2_blink  PROGMEM = { (1<<7) + 5,  AMP_BL, AMP_BL, AMP_BL };
*/


// Nachfolgend mal ein Baustellenblitzer
// LED0: ____x______________________________________x_____________________________
// LED1: _______x______________________________________x__________________________
// LED2: __________x______________________________________x_______________________
// LED3: _____________x______________________________________x____________________
// LED4: ________________x______________________________________x_________________
// LED5: ___________________x______________________________________x______________
// LED6: ______________________x______________________________________x___________
// LED7: _________________________x______________________________________x________



// #define BST_ON 1     // Time for one flash
// #define BST_OFF 80   // BST_ON + BST_OFF = period
// #define BST_RUN 4    // Offset between succeeding flashes
#define BST_ON  (  30000L / TICK_PERIOD)     // 30ms: Time for one flash
#define BST_OFF (1100000L / TICK_PERIOD)     // BST_ON + BST_OFF = period
#define BST_RUN ( 100000L / TICK_PERIOD)     // Offset between succeeding flashes

/*/                                        "ON" NR       rest, ontime, offtime
struct t_action port0_bst PROGMEM =   { (0<<7) + 0,   BST_RUN, BST_ON, BST_OFF};
struct t_action port1_bst PROGMEM =   { (0<<7) + 1, BST_RUN*2, BST_ON, BST_OFF};
struct t_action port2_bst PROGMEM =   { (0<<7) + 2, BST_RUN*3, BST_ON, BST_OFF};
struct t_action port3_bst PROGMEM =   { (0<<7) + 3, BST_RUN*4, BST_ON, BST_OFF};
struct t_action port4_bst PROGMEM =   { (0<<7) + 4, BST_RUN*5, BST_ON, BST_OFF};
struct t_action port5_bst PROGMEM =   { (0<<7) + 5, BST_RUN*6, BST_ON, BST_OFF};
struct t_action port6_bst PROGMEM =   { (0<<7) + 6, BST_RUN*7, BST_ON, BST_OFF};
struct t_action port7_bst PROGMEM =   { (0<<7) + 7, BST_RUN*8, BST_ON, BST_OFF};
*/

//------------------------------------------------------------------------------
// This Routine copies the given action values to the corresponding array
// (loading the values is a semaphor operation, therefore we disable int)

/*
void perform_action(const struct t_action *pData)
  {
    unsigned char ctrl;
    unsigned char port;
    unsigned char mask; 

    ctrl = pgm_read_byte(&pData->status);

    port = ctrl & 0b0111;
    mask = 1 << port;
    cli();
    if (ctrl & (1<<7)) PORTB |= mask;
    else               PORTB &= ~mask;
    
    out_pwm[port].rest    = pgm_read_byte(&pData->rest);
    out_pwm[port].ontime  = pgm_read_byte(&pData->ontime);
    out_pwm[port].offtime = pgm_read_byte(&pData->offtime);
    if (ctrl & (1<<6)) out_pwm[port].rest    = pulsdelay;  // override default
    sei(); 
  }

*/
//------------------------------------------------------------------------------
// This Routine is called when myAdr is received



void action(void)
  {
    unsigned char myCommand;
    
    if (ReceivedCommand & (1<<3))   // Bit 3: accessory command + active coil?
      {
        myCommand = ReceivedCommand & 0b00000111;

        cli();                      // block interrupts
        
        if (MyOpMode == 7)
          {
            Communicate |= (1<<C_DoSave); 
            // Traffic Light (Ampel)
            // Ampel
            // LED0:RT1 xxxxxxxx_____________________________xxxxxxxxxxxxxxxxx
            // LED1:GE1 ______xx___________________________xx_________________
            // LED2:GN1 ________xxxxxxxxxxxxxxxxxxxxxxxxxxx___________________
            // LED3:STOP1 xxxxxxx___________________________xxxxxxxxxxxxxxxxxx

            #define AMP_YE  (1000000L / TICK_PERIOD)   // Yellow 1 s
            #define AMP_BL  ( 500000L / TICK_PERIOD)   // Blinken
             if (myCommand == 0) // = schalte auf rot
               {
                      output(PB0,0);  // rot
                      out_pwm[0].rest    = AMP_YE;
                      out_pwm[0].ontime  = 0;
                    
                      output(PB1,1);
                      out_pwm[1].rest    = AMP_YE;
                      out_pwm[1].offtime = 0;
                    
                      output(PB2,0);   // grün
                      out_pwm[2].rest    = 0;
                      
                      output(PB3,0);   // StopMagnet nach halber Gelb ein
                      out_pwm[3].rest    = AMP_YE / 2;
                      out_pwm[3].ontime = 0;                   
               }
             else if (myCommand == 1) // = schalte auf grün
               {
                      output(PB0,1);  // rot
                      out_pwm[0].rest    = AMP_YE;
                      out_pwm[0].offtime  = 0;
                    
                      output(PB1,1);
                      out_pwm[1].rest    = AMP_YE;
                      out_pwm[1].offtime = 0;
                    
                      output(PB2,0);   // grün
                      out_pwm[2].rest    = AMP_YE;
                      out_pwm[2].ontime = 0;                      

                      output(PB3,1);   // Magnet nach 1,5 gelb aus
                      out_pwm[3].rest    = 3*AMP_YE / 2;
                      out_pwm[3].offtime = 0;                   
               }
             else if (myCommand == 2) // = alles aus
               {
                      output(PB1,0);
                      out_pwm[1].rest    = 0;
                      output(PB0,0);
                      out_pwm[0].rest    = 0;
                      output(PB2,0);
                      out_pwm[2].rest    = 0;
                      output(PB3,0);
                      out_pwm[3].rest    = 0;
               }                     
             else if (myCommand == 3) // blinken
               {
                      output(PB1,1);
                      out_pwm[1].rest    = AMP_YE;
                      out_pwm[1].ontime  = AMP_YE;
                      out_pwm[1].offtime = AMP_YE;
                      output(PB0,0);
                      out_pwm[0].rest    = 0;
                      output(PB2,0);
                      out_pwm[2].rest    = 0;
                      output(PB3,0);
                      out_pwm[3].rest    = 0;
               }
             else if (myCommand == 4) // = schalte auf rot
               {
                      output(PB4,0);  // rot
                      out_pwm[4].rest    = AMP_YE;
                      out_pwm[4].ontime  = 0;
                    
                      output(PB5,1);
                      out_pwm[5].rest    = AMP_YE;
                      out_pwm[5].offtime = 0;
                    
                      output(PB6,0);   // grün
                      out_pwm[6].rest    = 0;
                      
                      output(PB7,0);   // Magnet
                      out_pwm[7].rest    = AMP_YE / 2;
                      out_pwm[7].ontime = 0;                   
               }
             else if (myCommand == 5) // = schalte auf grün
               {
                      output(PB4,1);  // rot
                      out_pwm[4].rest    = AMP_YE;
                      out_pwm[4].offtime  = 0;
                    
                      output(PB5,1);
                      out_pwm[5].rest    = AMP_YE;
                      out_pwm[5].offtime = 0;
                    
                      output(PB6,0);   // grün
                      out_pwm[6].rest    = AMP_YE;
                      out_pwm[6].ontime = 0;                      

                      output(PB7,1);   // Magnet
                      out_pwm[7].rest    = 3*AMP_YE / 2;
                      out_pwm[7].offtime = 0;                   

               }
             else if (myCommand == 6) // = alles aus
               {
                      output(PB5,0);
                      out_pwm[5].rest    = 0;
                      output(PB4,0);
                      out_pwm[4].rest    = 0;
                      output(PB6,0);
                      out_pwm[6].rest    = 0;
                      output(PB7,0);
                      out_pwm[7].rest    = 0;
               }                     
             else // (myCommand == 7) // blinken
               {
                      output(PB5,1);
                      out_pwm[5].rest    = AMP_YE;
                      out_pwm[5].ontime  = AMP_YE;
                      out_pwm[5].offtime = AMP_YE;
                      output(PB4,0);
                      out_pwm[4].rest    = 0;
                      output(PB6,0);
                      out_pwm[6].rest    = 0;
                      output(PB7,0);
                      out_pwm[7].rest    = 0;
               }             
          }
          else if (MyOpMode == 6)
          {

             Communicate |= (1<<C_DoSave); 

             // MyOpMode6:
             // 2 x 4 Dauerausgänge für die Ansteuerung von 2 4-begriffigen Signale
             //   Adresse:   Ausgang:
             // ;   0        xxxx0001
             // ;   1        xxxx0010
             // ;   2        xxxx0100
             // ;   3        xxxx1000
             // ;   4        0001xxxx
             // ;   5        0010xxxx
             // ;   6        0100xxxx
             // ;   7        1000xxxx
;
            if (myCommand == 0)
              {
                    output(PB3,0);
                    output(PB2,0);
                    output(PB1,0);
                    output(PB0,1);
               }
            else if (myCommand == 1)
               {
                    output(PB3,0);
                    output(PB2,0);
                    output(PB1,1);
                    output(PB0,0);
               }
             else if (myCommand == 2)
               {
                    output(PB3,0);
                    output(PB2,1);
                    output(PB1,0);
                    output(PB0,0);
               }
             else if (myCommand == 3)
               {
                    output(PB3,1);
                    output(PB2,0);
                    output(PB1,0);
                    output(PB0,0);
               }
             else if (myCommand == 4)
               {
                    output(PB4,1);
                    output(PB5,0);
                    output(PB6,0);
                    output(PB7,0);
               }
             else if (myCommand == 5)
               {
                    output(PB4,0);
                    output(PB5,1);
                    output(PB6,0);
                    output(PB7,0);
               }
             else if (myCommand == 6)
               {
                    output(PB4,0);
                    output(PB5,0);
                    output(PB6,1);
                    output(PB7,0);
               }
             else // (myCommand == 7)
               {
                    output(PB4,0);
                    output(PB5,0);
                    output(PB6,0);
                    output(PB7,1);
               }       

          }
        else if (MyOpMode == 5)
          {

             // MyOpMode = 5:
             //   Kombination von 2 x 3 Dauerausgängen für 3-begriffige Signale
             //   und 1 x 2 Impulsausgängen 0,25 s
             //   die 3 begriffigen Signal erfordern noch externe Dioden!
             //   Adresse:   Ausgang:
             //   Adresse:   Bits
             //; 0       1 0 0 x x x x x
             //; 1       0 1 0 x x x x x
             //; 2       0 0 1 x x x x x
             //; 3       x x x 1 0 0 x x
             //; 4       x x x 0 1 0 x x
             //; 5       x x x 0 0 1 x x
             //; 6       x x x x x x 1 0
             //; 7       x x x x x x 0 1
            Communicate |= (1<<C_DoSave); 
            
            if (myCommand == 0)
              {
                    output(PB2,0);
                    output(PB1,0);
                    output(PB0,1);
                }
            else if (myCommand == 1)
               {
                    output(PB2,0);
                    output(PB1,1);
                    output(PB0,0);
               }
             else if (myCommand == 2)
               {
                    output(PB2,1);
                    output(PB1,0);
                    output(PB0,0);
                }
             else if (myCommand == 3)
               {
                    output(PB3,1);
                    output(PB4,0);
                    output(PB5,0);
                }
             else if (myCommand == 4)
               {
                    output(PB3,0);
                    output(PB4,1);
                    output(PB5,0);
               }
             else if (myCommand == 5)
               {
                    output(PB3,0);
                    output(PB4,0);
                    output(PB5,1);
               }
             else if (myCommand == 6)
               {
                    output(PB6,1);
                    output(PB7,0);
               }
             else // (myCommand == 7)
               {
                    output(PB6,0);
                    output(PB7,1);
               }       
          }
        else if (MyOpMode == 4)
          {  // permanent on
             Communicate |= (1<<C_DoSave); 
             
             if (myCommand == 0)
               {
                    output(PB1,0);         
                    output(PB0,1);
               }
             else if (myCommand == 1)
               {
                    output(PB1,1); 
                    output(PB0,0);
               }
             else if (myCommand == 2)
               {
                    output(PB2,1);
                    output(PB3,0);      
               }
             else if (myCommand == 3)
               {
                    output(PB2,0);
                    output(PB3,1);     
               }
             else if (myCommand == 4)
               {
                    output(PB4,1);
                    output(PB5,0);
                 }
             else if (myCommand == 5)
               {
                    output(PB4,0);
                    output(PB5,1);
               }
             else if (myCommand == 6)
               {
                    output(PB6,1);
                    output(PB7,0);
               }
             else // (myCommand == 7)
               {
                    output(PB6,0);
                    output(PB7,1);
               }           
          }
        else
          {   // all other
             if (myCommand == 0)
               {
                    out_pwm[0].rest = pulsdelay;         
                    out_pwm[1].rest = 0;         
                    output(PB0,1);
                    output(PB1,0);
               }
             else if (myCommand == 1)
               {
                    out_pwm[1].rest = pulsdelay;         
                    out_pwm[0].rest = 0;         
                    output(PB0,0);
                    output(PB1,1);
               }
             else if (myCommand == 2)
               {
                    out_pwm[2].rest = pulsdelay;         
                    out_pwm[3].rest = 0;         
                    output(PB2,1);
                    output(PB3,0);
              }
             else if (myCommand == 3)
               {
                    out_pwm[3].rest = pulsdelay;         
                    out_pwm[2].rest = 0;         
                    output(PB2,0);
                    output(PB3,1);
               }
             else if (myCommand == 4)
               {
                    out_pwm[4].rest = pulsdelay;         
                    out_pwm[5].rest = 0;         
                    output(PB4,1);
                    output(PB5,0);
               }
             else if (myCommand == 5)
               {
                    out_pwm[5].rest = pulsdelay;         
                    out_pwm[4].rest = 0;         
                    output(PB4,0);
                    output(PB5,1);
               }
             else if (myCommand == 6)
               {
                    out_pwm[6].rest = pulsdelay;         
                    out_pwm[7].rest = 0;         
                    output(PB6,1);
                    output(PB7,0);
               }
             else // (myCommand == 7)
               {
                    out_pwm[7].rest = pulsdelay;         
                    out_pwm[6].rest = 0;         
                    output(PB6,0);
                    output(PB7,1);
               }       
          }  // OpMode
        sei();
     }
  }

//==============================================================================
//
// Section 5
//
// MAIN: analyze command, call the action, do programming
//
//
//-----------------------------------------------------------------------------
// analyze_message checks teh received DCC message
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
                ReceivedCommand = message[1] & 0b00001111;
    
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
    
                MyAdr = (eeprom_read_byte(&EE_myAdrH) << 8) |
                        (eeprom_read_byte(&EE_myAdrL));
    
                if (ReceivedAdr == MyAdr) return(2);
                else return(1);
              }
          }
      }
    return(0);
  }


//------------------------------------------------------------------------
// This Routine is called when PROG is pressed
//
#define DEBOUNCE  (50000L / TICK_PERIOD)
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
                    eeprom_write_byte(&EE_myAdrL, (unsigned char) ReceivedAdr);     
                    eeprom_write_byte(&EE_myAdrH, (unsigned char) (ReceivedAdr >> 8));
                    
                    myCommand = ReceivedCommand & 0x07;
                    eeprom_write_byte(&EE_myOpMode, myCommand);
                    if (myCommand == 0)      pulsdelay =  300000L / TICK_PERIOD; 
                    else if (myCommand == 1) pulsdelay =  500000L / TICK_PERIOD; 
                    else if (myCommand == 2) pulsdelay = 1000000L / TICK_PERIOD; 
                    else                     pulsdelay = 2000000L / TICK_PERIOD; 
                    MyOpMode = myCommand;

                    eeprom_write_byte(&EE_Pulsdelay, pulsdelay);   
             
                    eeprom_write_byte(&EE_LastState, 0);
                    
                    do {} while (!eeprom_is_ready());    // wait for write to complete
                    
                    LED_OFF;

                    // we got reprogrammed ->
                    // forget everthing running and restart decoder!                    
                    
                    cli();
                    
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


//------------------------------------------------------------------------

int main(void)
  {
    init_main();
     
     // Lese die eigene DCC-Adresse und Betriebsart

    // MyAdr = (eeprom_read_byte(&EE_myAdrH) << 8) |
    //         (eeprom_read_byte(&EE_myAdrL));

    pulsdelay = eeprom_read_byte(&EE_Pulsdelay);   

    MyOpMode  = eeprom_read_byte(&EE_myOpMode);
     
    if (MyOpMode < 4)
      {  // Modes 0,1,2,3 are pulse modes - no last state
        #if (INVERT_OUTPUT == 1)
          PORTB = 0xFF;
        #else
          PORTB = 0;
        #endif 
      }
    else
      {
        PORTB = eeprom_read_byte(&EE_LastState);
      }

    Communicate = 0; 
    Recstate = 1<<RECSTAT_WF_PREAMBLE;  

        #if (SIMULATION == 2)
          // run port 0 fast blinking
          // run port 1 slow blinking
          PORTB = 0b11;
          out_pwm[0].rest    = 100000L / TICK_PERIOD;
          out_pwm[0].ontime  = 100000L / TICK_PERIOD;
          out_pwm[0].offtime = 100000L / TICK_PERIOD;
          out_pwm[1].rest    = 200000L / TICK_PERIOD;
          out_pwm[1].ontime  = 200000L / TICK_PERIOD;
          out_pwm[1].offtime = 200000L / TICK_PERIOD;
        #endif

        #if (SIMULATION == 3)
          // Baustellenblitzer
          PORTB = 0;
          unsigned char i;
          for (i=0; i<8; i++)
            { out_pwm[i].rest    = BST_RUN + BST_RUN * i;
              out_pwm[i].ontime  = BST_ON;
              out_pwm[i].offtime = BST_OFF;
            }
        #endif


        #if (SIMULATION == 1)
          simulat_receive();
        #endif

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

        cli();
        if (Communicate & (1<<C_DoSave) )
          {
            sei();
            Communicate &= ~(1<<C_DoSave);
            if (JUMPER_FITTED)
              {
                eeprom_write_byte(&EE_LastState, PORTB);   
              } 
          } 
        sei();

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
                eeprom_write_byte(&EE_LastState, PORTB);   
              } 
          } 
        // if (PROG_PRESSED) DoProgramming();         
      }
 
  }


#endif
