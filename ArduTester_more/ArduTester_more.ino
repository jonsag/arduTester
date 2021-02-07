


  #include <avr/io.h>
	#include <util/delay.h>
	#include <avr/sleep.h>
	#include <stdlib.h>
  #include "stdint.h" //JLG
	#include <string.h>
	#include <avr/eeprom.h>
	#include <avr/pgmspace.h>
	#include <avr/wdt.h>
	#include <math.h>
	
	#define MAIN_C

	#include "Makefile.h" //JLG
	#include "Transistortester.h"
	#include "config.h"
	#include "part_defs.h"
	#include "lcd_routines.h"
	#include "lcd_defines.h"
	#include "wait1000ms.h"
	#include "autoconf.h"
	#include "tt_function.h"
  #include "tt_resistor.h"
  #include "font.h"
  #include "24x32update_bitmaps.h"

	
/* defines global variables in RAM and EEprom from file tt_globals.h */
/* removed ifdef MAIN_C because always defined */
#define COMMON
#ifdef AUTO_CAL
  const int8_t RefDiff EEMEM = REF_R_KORR;	// correction of internal Reference Voltage

//  const uint16_t cap_null EEMEM = C_NULL;	// Zero offset of capacity measurement 
   const int16_t ref_offset EEMEM = REF_C_KORR;	// default correction of internal reference voltage for capacity measurement
   // the zero offset for capacity measurement for all pin combinations
  // LoPin:HiPin                        2:1    3:1    1:2                    marker  3:2                   1:3    2:3
   const uint8_t c_zero_tab[] EEMEM = { C_NULL,C_NULL,C_NULL+TP2_CAP_OFFSET,C_NULL+2,C_NULL+TP2_CAP_OFFSET,C_NULL,C_NULL }; //table of zero offsets
   // if the marker position of c_zero_tab is not equal the first position, the calibration has not run before
#endif
#ifdef SamplingADC
   const uint16_t c_zero_tab2_lo[] EEMEM = { C_NULL*100,C_NULL*100,C_NULL*100,C_NULL*100,C_NULL*100,C_NULL*100,C_NULL*100 };   // zero offsets for SamplingADC capacitance measurement, in 0.01 pF, lo voltage
   const uint16_t c_zero_tab2_hi[] EEMEM = { C_NULL*100,C_NULL*100,C_NULL*100,C_NULL*100,C_NULL*100,C_NULL*100,C_NULL*100 };   // same, hi voltage
#endif
#ifdef WITH_MENU
   const int8_t big_cap_corr EEMEM = C_H_KORR;	// default correction for big capacity measurement
 #if defined(WITH_FREQUENCY_DIVIDER) && !defined(NO_FREQ_COUNTER)
  #ifndef FREQ_SCALER
   #define FREQ_SCALER 0		// 1<<0 = 1
  #endif
  const uint8_t f_scaler EEMEM = FREQ_SCALER;	// default scaler for frequency measurement
 #endif
#endif

  const uint8_t EE_ESR_ZEROtab[] EEMEM = {ESR_ZERO, ESR_ZERO, ESR_ZERO, ESR_ZERO};	// zero offset of ESR measurement
#ifdef WITH_ROTARY_SWITCH
//  const uint8_t EE_RotarySwitch EEMEM = 0;	// rotation switch is not detected
#endif
#if ((LCD_ST_TYPE == 7565) || (LCD_ST_TYPE == 1306) || (LCD_ST_TYPE == 8812) || (LCD_ST_TYPE == 8814) || defined(LCD_DOGM))
  const uint8_t EE_Volume_Value EEMEM = VOLUME_VALUE;	// Volume Value for ST7565 controller 
#endif
#ifdef LCD_CHANGE_COLOR
  const uint8_t EE_BG_COLOR1 EEMEM = LCD_BG_COLOR & 0xff;	// lower bits of background color
  const uint8_t EE_BG_COLOR2 EEMEM = LCD_BG_COLOR >> 8;		// higher bits of background color 
  const uint8_t EE_FG_COLOR1 EEMEM = LCD_FG_COLOR & 0xff;	// lower bits of foreground color
  const uint8_t EE_FG_COLOR2 EEMEM = LCD_FG_COLOR >> 8;		// higher bits of foreground color 
#endif

struct Diode_t {
  uint8_t Anode[6];
  uint8_t Cathode[6];
  unsigned int Voltage[6];
};
COMMON struct Diode_t diodes;

struct Switch_t {

  union {
  unsigned long Pw;	// combined Mask
  uint8_t R[4];	// mask to switch a Pin with R_L,  mask to switch a Pin with R_H
  } Pin;

};

COMMON uint8_t NumOfDiodes;
COMMON uint8_t diode_sequence;

typedef struct {
  unsigned long hfe;		//current amplification factor 
  unsigned int uBE;		//B-E-voltage of the Transistor or RDS for E-MOS; Idss for JFET
  unsigned int current;		// current of Drain in 1uA
  unsigned int ice0;		// for BJT ICEO in 1uA; for FET cut-off voltage in mV
  unsigned int gthvoltage;	//Gate-threshold voltage 
  				// for bipolar gthvoltage is ICEs in 1uA
 #define ices gthvoltage
	// note: don't change the total size of the above fields, since the offsets of the following 3 fields are hard-coded in PinLayout.S
  uint8_t b,c,e;		//pins of the Transistor
  uint8_t count;
}trans_t;

COMMON trans_t ptrans;		// parameters of P type transistor
COMMON trans_t ntrans;		// parameters of N type transistor
COMMON trans_t *_trans;		// pointer to trans_t structure

COMMON uint8_t tmpval, tmpval2;
COMMON unsigned int ref_mv;      //Reference-voltage  in mV units (as read with ADC)
COMMON unsigned int ref_mv_offs;       //Reference-voltage  in mV units with eeprom offset for C
COMMON unsigned int adc_internal_reference;  //internal reference voltage of ADC in mV units
COMMON unsigned int adc_vcc_reference;  // reference voltage of ADC,if switched to VCC in mV units
COMMON  unsigned int RHmultip;	// Multiplier for capacity measurement with R_H (470KOhm)

#ifdef WITH_MENU
COMMON union t_frq{
  unsigned long dw;
  uint16_t w[2];
  uint8_t b[4];
} ext_freq;	// external frequency
//COMMON unsigned long ext_period; 
COMMON unsigned int pinchange_count;
COMMON unsigned int pinchange_max;
#endif

COMMON struct cap_t {
  // Attention! If you change this structure, you must also change defines in GetESR.S !!!!
  unsigned long cval;		// capacitor value 
  unsigned long cval_max;	//capacitor with maximum value
  union t_combi{
  unsigned long dw;	// capacity value without corrections
  uint16_t w[2];
  } cval_uncorrected;

#if FLASHEND > 0x1fff
  unsigned int esr;		// serial resistance of C in 0.01 Ohm
  unsigned int v_loss;		// voltage loss 0.1%
#endif

  uint8_t ca, cb;		//pins of capacitor
  int8_t cpre;			//Prefix for capacitor value  -12=p, -9=n, -6=µ, -3=m
  int8_t cpre_max;		//Prefix of the biggest capacitor
} cap;
  unsigned int cell_mv[3];	//remaining load voltages after discharge cycle

#ifndef INHIBIT_SLEEP_MODE
 /* with sleep mode we need a global ovcnt16 */
COMMON volatile uint16_t ovcnt16;
COMMON volatile uint8_t unfinished;
#endif

COMMON int16_t load_diff;		// difference voltage of loaded capacitor and internal reference

COMMON uint8_t WithReference;		// Marker for found precision voltage reference = 1
COMMON uint8_t PartFound;	 	// type of the found part 
COMMON uint8_t PartMode;		// description of the found part
COMMON char outval[10];			// String for ASCII-output i2lcd, u2lcd
//COMMON char OutBuffer[10];		// String for ASCII-output DisplayValue
COMMON uint8_t empty_count;		// counter for max count of empty measurements
COMMON uint8_t mess_count;		// counter for max count of nonempty measurements

COMMON struct ADCconfig_t {
  uint8_t Samples;		// number of ADC samples to take
  uint8_t RefFlag;		// save Reference type VCC of IntRef
  uint16_t U_Bandgap;		// Reference Voltage in mV
  uint16_t U_AVCC;		// Voltage of AVCC
} ADCconfig;

#ifdef AUTO_CAL
COMMON uint8_t pin_combination;		// coded Pin-combination  2:1,3:1,1:2,x:x,3:2,1:3,2:3
COMMON uint16_t resis680pl;	// port output resistance + 680
COMMON uint16_t resis680mi;	// port output resistance + 680
COMMON uint16_t pin_rmi;	// port output resistance to GND side, 0.1 Ohm units
COMMON uint16_t pin_rpl;	// port output resistance to VCC side, 0.1 Ohm units
COMMON uint8_t UnCalibrated;	// 0, if the tester is calibrated
#endif

#ifdef WITH_ROTARY_SWITCH
 #define ROT_MSK 0x03		/* must be power of two - 1: 3,7,15 */
struct Rotary_t {
 uint8_t state[(ROT_MSK+1)];	// coded state history of the rotatry switch, bit 0 == state of A-switch, bit 1 = state of B-switch
 uint8_t ind;		// index to the last entry of the state history (rotary switch)
 int8_t count;		// count of right steps, negative if left steps
 uint8_t incre;		// absolute value of step count
 #if WITH_ROTARY_SWITCH == 4
 // no rotary switch connected, UP and DOWN  key is present
 uint8_t a_state;	// history of switch A state for single UP switch
 uint8_t b_state;	// history of switch B state for single DOWN switch
 #endif
};
COMMON struct Rotary_t rotary;
COMMON uint8_t rotary_switch_present;	// is set to 1, if rotary switch movement is detected
// COMMON  const uint8_t EE_RotarySwitch; 	// rotation switch is detected
#endif
#if FLASHEND > 0x1fff
COMMON uint8_t DC_Pwr_mode;
#endif

COMMON uint8_t lcd_text_line;
COMMON uint8_t _lcd_column;

COMMON uint8_t last_line_used;
//#if POWER_OFF+0 > 1
COMMON unsigned int display_time;	// display time of measurement in ms units
//#endif
#if (LCD_ST_TYPE == 7920)
COMMON uint8_t lcd_bit_mem[64][16];
#endif
#if ((LCD_ST_TYPE == 7565) || (LCD_ST_TYPE == 1306) || defined(LCD_DOGM))
 //COMMON const uint8_t EE_Volume_Value EEMEM;   // Volume Value for ST7565 controller 
#endif
#ifdef LCD_CHANGE_COLOR
COMMON union {
  uint16_t w;
  uint8_t b[2];
} lcd_bg_color;
#endif
#if defined(LCD_CHANGE_COLOR) || defined(LCD_ICON_COLOR)
COMMON union {
  uint16_t w;
  uint8_t b[2];
} lcd_fg_color;
#endif
#if defined(LCD_ICON_COLOR)
COMMON union {
  uint16_t w;
  uint8_t b[2];
} lcd_fg2_color;
#endif
/* END defines global variables in RAM and EEprom from file tt_globals.h */


	#ifndef INHIBIT_SLEEP_MODE
	  // prepare sleep mode
	  EMPTY_INTERRUPT(TIMER2_COMPA_vect);
	#endif
        #if !defined(INHIBIT_SLEEP_MODE) || defined(SamplingADC)
	  // ADC_vect is always required by samplingADC()
	  EMPTY_INTERRUPT(ADC_vect);
        #endif




/**************************************************
***      begin of transistortester program
***************************************************/
void setup() {
  
#ifdef WITH_HARDWARE_SERIAL
 Serial.begin(115200); // Serial Monitor
#endif

//#ifdef ARDUINO_UNO
// pinMode(A3,INPUT_PULLUP);// avoids external 10K Pullup
//#endif
	  uint8_t ii;
	  unsigned int max_time;
	#ifdef SEARCH_PARASITIC
	  unsigned long n_cval;		// capacitor value of NPN B-E diode, for deselecting the parasitic Transistor
	  int8_t n_cpre;		// capacitor prefix of NPN B-E diode
	#endif
	#ifdef WITH_GRAPHICS
	  unsigned char options;
	#endif
        uint8_t vak_diode_nr;		// number of the protection diode of BJT
        union {
        uint16_t pw;
        uint8_t pb[2];
        } rpins;
        uint8_t x, y, z;
	  //switch on
	  ON_DDR = (1<<ON_PIN);			// switch to output
	  ON_PORT = (1<<ON_PIN); 		// switch power on 
	#ifndef PULLUP_DISABLE
	  RST_PORT |= (1<<RST_PIN); 	// enable internal Pullup for Start-Pin
	#endif
	  uint8_t tmp;
	  //ADC-Init
	  ADCSRA = (1<<ADEN) | AUTO_CLOCK_DIV;	//prescaler=8 or 64 (if 8Mhz clock)
	#ifdef __AVR_ATmega8__
	// #define WDRF_HOME MCU_STATUS_REG
	 #define WDRF_HOME MCUCSR
	#else
	 #define WDRF_HOME MCUSR
	 #if FLASHEND > 0x3fff
	  // probably was a bootloader active, disable the UART
	  UCSR0B = 0;		// disable UART, if started with bootloader
	 #endif
	#endif
          wait500ms();

	#if (PROCESSOR_TYP == 644) || (PROCESSOR_TYP == 1280)
	 #define BAUD_RATE 9600
//	  UBRR0H = (F_CPU / 16 / BAUD_RATE - 1) >> 8;
//	  UBRR0L = (F_CPU / 16 / BAUD_RATE - 1) & 0xff;
//	  UCSR0B = (1<<TXEN0);
//	  UCSR0C = (1<<USBS0) | (3<<UCSZ00);	// 2 stop bits, 8-bit
//	  while (!(UCSR0A & (1<<UDRE0))) { };	// wait for send data port ready 
         #ifdef SWUART_INVERT
	  SERIAL_PORT &= ~(1<<SERIAL_BIT);
         #else
	  SERIAL_PORT |= (1<<SERIAL_BIT);
         #endif
          SERIAL_DDR |= (1<<SERIAL_BIT);
	#endif

	  tmp = (WDRF_HOME & ((1<<WDRF)));	// save Watch Dog Flag
	  WDRF_HOME &= ~(1<<WDRF);	 	//reset Watch Dog flag
	  wdt_disable();			// disable Watch Dog
	#ifndef INHIBIT_SLEEP_MODE
	  // switch off unused Parts
	 #if PROCESSOR_TYP == 644
          #ifdef PRUSART1
	  PRR0 = (1<<PRTWI) |  (1<<PRSPI) | (1<<PRUSART1);
          #else
	  PRR0 = (1<<PRTWI) |  (1<<PRSPI) ;
          #endif
	//  PRR1 =  (1<<PRTIM3) ;
	 #elif PROCESSOR_TYP == 1280
	  PRR0 = (1<<PRTWI) |  (1<<PRSPI) | (1<<PRUSART1);
	  PRR1 = (1<<PRTIM5) | (1<<PRTIM4) | (1<<PRTIM3) | (1<<PRUSART3) | (1<<PRUSART2) | (1<<PRUSART3);
	 #else
	  PRR = (1<<PRTWI) | (1<<PRSPI) | (1<<PRUSART0);
	 #endif
//	disable digital inputs of Analog pins, but TP1-3 digital inputs must be left enabled for VGS measurement
	  DIDR0 = ((1<<ADC5D) | (1<<ADC4D) | (1<<ADC3D) | (1<<ADC2D) | (1<<ADC1D) | (1<<ADC0D)) & ~((1<<TP3) | (1<<TP2) | (1<<TP1));	
	  TCCR2A = (0<<WGM21) | (0<<WGM20);		// Counter 2 normal mode
	  TCCR2B = CNTR2_PRESCALER;	//prescaler set in autoconf
	#endif		/* INHIBIT_SLEEP_MODE */
	  sei();				// enable interrupts
	  lcd_init();				//initialize LCD
		
	//  ADC_PORT = TXD_VAL;
	//  ADC_DDR = TXD_MSK;
	  if(tmp) { 
	     // check if  Watchdog-Event 
	     // this happens, if the Watchdog is not reset for 2s
	     // can happen, if any loop in the Program doen't finish.
	     lcd_line1();
             lcd_MEM_string(TestTimedOut);	//Output Timeout
	     wait_about3s();			// time to read the Timeout message
	     switch_tester_off();
	     return 0;
	  }

	#ifdef PULLUP_DISABLE
	 #ifdef __AVR_ATmega8__
	  SFIOR = (1<<PUD);		// disable Pull-Up Resistors mega8
	 #else
	  MCUCR = (1<<PUD);		// disable Pull-Up Resistors mega168 family
	 #endif
	#endif

	//#if POWER_OFF+0 > 1
	  // tester display time selection
	#ifndef USE_EEPROM
	  EE_check_init();		// init EEprom, if unset
	#endif
	#ifdef WITH_ROTARY_SWITCH
	//  rotary_switch_present = eeprom_read_byte(&EE_RotarySwitch);
	  rotary.ind = ROT_MSK+1;		//initilize state history with next call of check_rotary()
	#endif
#ifdef WITH_HARDWARE_SERIAL
//	ii = 60;
	ii = 30;
#else
	#if 1
	  for (ii=0; ii<60; ii++) {
		if (RST_PIN_REG & (1 << RST_PIN))
			break;	// button is released
	     wait_about10ms();
	  }
	#else
	  ii = 0;
	  if (!(RST_PIN_REG & (1<<RST_PIN))) {
	     // key is still pressed
	     ii = wait_for_key_ms(700);	
	  }
	#endif
	  display_time = OFF_WAIT_TIME;		// LONG_WAIT_TIME for single mode, else SHORT_WAIT_TIME
	  if (ii > 30) {
	     display_time = LONG_WAIT_TIME;	// ... set long time display anyway
	  }
#endif // WITH_HARDWARE_SERIAL
	#if POWER_OFF+0 > 1
	  empty_count = 0;
	  mess_count = 0;
	#endif
	  ADCconfig.RefFlag = 0;
	  Calibrate_UR();		// get Ref Voltages and Pin resistance
	#ifdef WDT_enabled
	  wdt_enable(WDTO_2S);		//Watchdog on
	#endif
	#ifdef WITH_MENU
	  if (ii >= 60) {
		while(function_menu());		// selection of function
	  }
	#endif

	//*****************************************************************
	//Entry: if start key is pressed before shut down
	loop_start:
	#if ((LCD_ST_TYPE == 7565) || (LCD_ST_TYPE == 1306))
	  lcd_command(CMD_DISPLAY_ON);
	  lcd_command(CMD_SET_ALLPTS_NORMAL);		// 0xa4
	#endif
	  lcd_clear();			// clear the LCD
	  ADC_DDR = TXD_MSK;		// activate Software-UART 
          init_parts();			// reset parts info to nothing found
	  Calibrate_UR();		// get Ref Voltages and Pin resistance
	  lcd_line1();			// Cursor to 1. row, column 1
	  
	#ifdef BAT_CHECK
	  // Battery check is selected
        Battery_check();
	#else
	  lcd_MEM_string(VERSION_str);		// if no Battery check, Version .. in row 1
	#endif	/* BAT_CHECK */

	  // begin tests
	#if FLASHEND > 0x1fff
	  if (WithReference) {
	     /* 2.5V precision reference is checked OK */
	 #if POWER_OFF+0 > 1
	     if ((mess_count == 0) && (empty_count == 0))
	 #endif
	     {
		 /* display VCC= only first time */
		 lcd_line2();
		 lcd_MEM_string(VCC_str);		// VCC=
		 Display_mV(ADCconfig.U_AVCC,3);	// Display 3 Digits of this mV units
		 lcd_refresh();			// write the pixels to display, ST7920 only
		 wait_about1s();		// time to read the VCC= message
	     }
	  }
	#endif
	#ifdef WITH_VEXT
	  unsigned int Vext;
	  // show the external voltage
	  while (!(RST_PIN_REG & (1<<RST_PIN))) {
	     lcd_clear_line2();
	     lcd_MEM_string(Vext_str);		// Vext=
	     ADC_DDR = 0;		//deactivate Software-UART
	     Vext = W5msReadADC(TPext);	// read external voltage 
	//     ADC_DDR = TXD_MSK;		//activate Software-UART 
	    uart_newline();		// MAURO replaced uart_putc(' ') by uart_newline(), 'Z'
	 #if EXT_NUMERATOR <= (0xffff/U_VCC)
	     Display_mV(Vext*EXT_NUMERATOR/EXT_DENOMINATOR,3);	// Display 3 Digits of this mV units
	 #else
             DisplayValue((unsigned long)Vext*EXT_NUMERATOR/EXT_DENOMINATOR,-3,'V',3);  // Display 3 Digits of this mV units
	 #endif
	     lcd_refresh();		// write the pixels to display, ST7920 only
	     wait_about300ms();		// delay to read the Vext= message
	  }
	#endif /* WITH_VEXT */

	#ifndef DebugOut
	  lcd_line2();			//LCD position row 2, column 1
	#endif
	  EntladePins();		// discharge all capacitors!
	  if(PartFound == PART_CELL) {
	    lcd_clear();
	    lcd_MEM_string(Cell_str);	// display "Cell!"
	#if FLASHEND > 0x3fff
	    lcd_line2();		// use LCD line 2
	    Display_mV(cell_mv[0],3);
	    lcd_space();
	    Display_mV(cell_mv[1],3);
	    lcd_space();
	    Display_mV(cell_mv[2],3);
	#endif
	#ifdef WITH_SELFTEST
	    lcd_refresh();			// write the pixels to display, ST7920 only
	    wait_about2s();
	    AutoCheck(0x11);		// full Selftest with "Short probes" message
	#endif
	    goto tt_end;
	  }

	#ifdef WITH_SELFTEST
	 #ifdef AUTO_CAL
	  lcd_cursor_off();
	  UnCalibrated = (eeprom_read_byte(&c_zero_tab[3]) - eeprom_read_byte(&c_zero_tab[0]));
	  if (UnCalibrated != 0) {
	     // if calibrated, both c_zero_tab values are identical! c_zero_tab[3] is not used otherwise
	     lcd_cursor_on();
	  }
	 #endif
	 #ifdef WITH_MENU
	  AutoCheck(0x00);			//check, if selftest should be done, only calibration
	 #else
	  AutoCheck(0x01);			//check, if selftest should be done, full selftest without MENU
	 #endif
	#endif
	#if FLASHEND > 0x1fff
          lcd_clear_line2();			//LCD position row2, column 1
        #else
          lcd_line2();				//LCD position row2, column 1
        #endif
	  lcd_MEM_string(TestRunning);		//String: testing...
	  lcd_refresh();			// write the pixels to display, ST7920 only
	 #ifdef WITH_UART
	    uart_putc(0x03);		// ETX, start of new measurement 
	    uart_newline();			 // MAURO Added
	 #endif
//
	  // check all 6 combinations for the 3 pins 
	//         High  Low  Tri
	  CheckPins(TP1, TP2, TP3);
	  CheckPins(TP2, TP1, TP3);

	  CheckPins(TP1, TP3, TP2);
	  CheckPins(TP3, TP1, TP2);

	  CheckPins(TP2, TP3, TP1);
	  CheckPins(TP3, TP2, TP1);

	  // Capacity measurement is only possible correctly with two Pins connected.
	  // A third connected pin will increase the capacity value!
	//  if(((PartFound == PART_NONE) || (PartFound == PART_RESISTOR) || (PartFound == PART_DIODE)) ) {
	  if(PartFound == PART_NONE) {
	     // If no part is found yet, check separate if is is a capacitor
#ifdef DebugOut
	     lcd_data('C');
#endif
	     EntladePins();		// discharge capacities
	     //measurement of capacities in all 3 combinations
	     ReadCapacity(TP3, TP1);
#ifdef DebugOut
	     lcd_data('K');
#endif
	#if DebugOut != 10
	     ReadCapacity(TP3, TP2);
#ifdef DebugOut
	     lcd_data('K');
#endif
	     ReadCapacity(TP2, TP1);
#ifdef DebugOut
	     lcd_data('K');
#endif
	#endif
	  }

#ifdef WITH_UJT
// check for UJT
        if (PartFound==PART_DIODE       
            && NumOfDiodes==2                // UJT is detected as 2 diodes E-B1 and E-B2...
//            && ResistorsFound==1             // ...and a resistor B1-B2
            && diodes.Anode[0]==diodes.Anode[1]      // check diodes have common anode
//            && (unsigned char)(ResistorList[0]+diodes.Anode[0])==2    // and resistor is between cathodes
           ) 
           // note: there also exist CUJTs (complementary UJTs); they seem to be (even) rarer than UJTs, and are not supported for now
           {
            CheckUJT();
        }
#endif		/* defined WITH_UJT */

#ifdef WITH_XTAL
        if (PartFound==PART_NONE || ((PartFound==PART_CAPACITOR) && (cap.cpre_max == -12))) {
           // still not recognized anything? then check for ceramic resonator or crystal
           // these tests are time-consuming, so we do them last, and only on TP1/TP3
           sampling_test_xtal();
        }
#endif

	  //All checks are done, output result to display

	#ifdef DebugOut 
	  // only clear two lines of LCD

	  lcd_clear_line1();
	#else
	  lcd_clear();				// clear total display
	#endif

	  _trans = &ntrans;			// default transistor structure to show
	  if (PartFound == PART_THYRISTOR) {
#ifdef WITH_GRAPHICS
            lcd_big_icon(THYRISTOR|LCD_UPPER_LEFT);
            lcd_draw_trans_pins(-8, 16);
            lcd_set_cursor(0,TEXT_RIGHT_TO_ICON);		// position behind the icon, Line 1
	    lcd_MEM_string(Thyristor);		//"Thyristor"
#else
	    lcd_MEM_string(Thyristor);		//"Thyristor"
            PinLayout(Cathode_char,'G','A'); 	// CGA= or 123=...
#endif
            goto TyUfAusgabe;
          }

  if (PartFound == PART_TRIAC) {
#ifdef WITH_GRAPHICS
    lcd_big_icon(TRIAC|LCD_UPPER_LEFT);
    lcd_draw_trans_pins(-8, 16);
    lcd_set_cursor(0,TEXT_RIGHT_TO_ICON);		// position behind the icon, Line 1
    lcd_MEM_string(Triac);		//"Triac"
#else
    lcd_MEM_string(Triac);		//"Triac"
    PinLayout('1','G','2'); 	// CGA= or 123=...
#endif
    goto TyUfAusgabe;
  }

#ifdef WITH_PUT
   if (PartFound == PART_PUT) {
      static const unsigned char PUT_str[] MEM_TEXT = "PUT";
      lcd_MEM_string(PUT_str);
      _trans=&ptrans;
      PinLayout('A','G',Cathode_char);
      goto TyUfAusgabe;
   }
#endif

#ifdef WITH_UJT
   if (PartFound == PART_UJT) {
      static const unsigned char UJT_str[] MEM_TEXT = "UJT";
      lcd_MEM_string(UJT_str);
      PinLayout('1','E','2');
 #ifdef SamplingADC
      static const unsigned char eta_str[] MEM_TEXT = " eta=";
      lcd_next_line(0);
      ResistorChecked[ntrans.e - TP_MIN + ntrans.c - TP_MIN - 1] = 0;	// forget last resistance measurement
      GetResistance(ntrans.c, ntrans.e);	// resistor value is in ResistorVal[resnum]
      DisplayValue(ResistorVal[ntrans.e - TP_MIN + ntrans.c - TP_MIN - 1],-1,LCD_CHAR_OMEGA,2);
      lcd_MEM_string(eta_str);		//"eta="
      DisplayValue(ntrans.gthvoltage,0,'%',3);
 #else /* ! SamplingADC */
      static const unsigned char R12_str[] MEM_TEXT = "R12=";
      lcd_next_line(0);
      lcd_MEM_string(R12_str);		//"R12="
      DisplayValue(ResistorVal[ntrans.e - TP_MIN + ntrans.c - TP_MIN - 1],-1,LCD_CHAR_OMEGA,2);
      lcd_data(',');
      DisplayValue(((RR680PL * (unsigned long)(ADCconfig.U_AVCC - ntrans.uBE)) / ntrans.uBE)-RRpinPL,-1,LCD_CHAR_OMEGA,3);
 #endif	 /* SamplingADC */
      goto tt_end;
   }
#endif /* WITH_UJT */

  if (PartFound == PART_CAPACITOR) {
#if FLASHEND > 0x3fff
     if ((cap.ca + cap.cb) == (TP1 + TP3)) {
        show_Cap13();		// repeated capacity measurement
        goto shut_off;		// key was pressed or timeout
     }
     show_cap(0);		// show capacity in normal way and measure additional parameters
#else
     show_cap_simple();		// show capacity in normal way and measure additional parameters
#endif
     goto tt_end;
  } /* end PartFound == PART_CAPACITOR */

#ifdef WITH_XTAL
  if (PartFound == PART_CERAMICRESONATOR) {
//      static const unsigned char cerres_str[] MEM_TEXT = "Cer.resonator  ";
      lcd_MEM_string(cerres_str);
      if (sampling_measure_xtal()) goto loop_start;
      goto tt_end;
  }
  if (PartFound == PART_XTAL) {
//      static const unsigned char xtal_str[] MEM_TEXT = "Crystal  ";
      lcd_MEM_string(xtal_str);
      if (sampling_measure_xtal()) goto loop_start;
      goto tt_end;
  }
#endif

  // ========================================
  if(PartFound == PART_DIODE) {
  // ========================================
     if(NumOfDiodes == 1) {		//single Diode
//        lcd_MEM_string(Diode);		//"Diode: "
#if FLASHEND > 0x1fff
        // enough memory (>8k) to sort the pins and additional Ir=
        DiodeSymbol_withPins(0);
	GetIr(diodes.Cathode[0],diodes.Anode[0]);	// measure and output Ir=x.xuA
#else
        // too less memory to sort the pins
        DiodeSymbol_withPins(0);
#endif
        UfAusgabe(0x70);		// mark for additional resistor and output Uf= in line 2
#ifndef SamplingADC
        /* load current of capacity is (5V-1.1V)/(470000 Ohm) = 8298nA */
        ReadCapacity(diodes.Cathode[0],diodes.Anode[0]);	// Capacity opposite flow direction
        if (cap.cpre < -3) {	/* capacity is measured */
 #if (LCD_LINES > 2)
           lcd_line3();		// output Capacity in line 3
 #endif
           lcd_MEM_string(Cap_str);	//"C="
 #if LCD_LINE_LENGTH > 16
           DisplayValue(cap.cval,cap.cpre,'F',3);
 #else
           DisplayValue(cap.cval,cap.cpre,'F',2);
 #endif
        }
#else  // SamplingADC
showdiodecap:
        cap.cval=sampling_cap(diodes.Cathode[0],diodes.Anode[0],0);   // at low voltage
        lcd_next_line_wait(0);		// next line, wait 5s and clear line 2
        DisplayValue(cap.cval,sampling_cap_pre,'F',2);
 #ifdef PULLUP_DISABLE
        lcd_data('-');
        cap.cval=sampling_cap(diodes.Cathode[0],diodes.Anode[0],1);   // at high voltage
        if (cap.cval < 0) cap.cval = 0;		// don't show negativ value
        DisplayValue(cap.cval,sampling_cap_pre,'F',2);
  #if LCD_LINE_LENGTH > 16
        lcd_MEM_string(AT05volt);	// " @0-5V"
  #else
        lcd_MEM_string(AT05volt+1);	// "@0-5V"
  #endif
        uart_newline();			// MAURO Diode ('A')
 #else
  #warning Capacity measurement from high to low not possible for diodes without PULLUP_DISABLE option!
 #endif  /* PULLUP_DISABLE */
#endif
        goto end3;
     } else if(NumOfDiodes == 2) { // double diode
        lcd_data('2');
        lcd_MEM_string(Dioden);		//"diodes "
        if(diodes.Anode[0] == diodes.Anode[1]) { //Common Anode
           DiodeSymbol_CpinApin(0);	// 1-|<-2
           DiodeSymbol_ACpin(1);	//  ->|-3
           UfAusgabe(0x01);
#ifdef SamplingADC
           goto showdiodecap;   // double diodes are often varicap; measure capacitance of one of them
#else
           goto end3;
#endif
        } 
        if(diodes.Cathode[0] == diodes.Cathode[1]) { //Common Cathode
           DiodeSymbol_ApinCpin(0);	// 1->|-2
           DiodeSymbol_CApin(1);	//  -|<-3
           UfAusgabe(0x01);
#ifdef SamplingADC
           goto showdiodecap;   // double diodes are often varicap; measure capacitance of one of them
#else
           goto end3;
#endif
//        else if ((diodes.Cathode[0] == diodes.Anode[1]) && (diodes.Cathode[1] == diodes.Anode[0])) 
        } 
        if (diodes.Cathode[0] == diodes.Anode[1]) {
           // normaly two serial diodes are detected as three diodes, but if the threshold is high
           // for both diodes, the third diode is not detected.
           // can also be Antiparallel
           diode_sequence = 0x01;	// 0 1
           SerienDiodenAusgabe();
           goto end3;
        } 
        if (diodes.Cathode[1] == diodes.Anode[0]) {
           diode_sequence = 0x10;	// 1 0
           SerienDiodenAusgabe();
           goto end3;
        }
     } else if(NumOfDiodes == 3) {
        //Serial of 2 Diodes; was detected as 3 Diodes 
        diode_sequence = 0x33;	// 3 3
        /* Check for any constellation of 2 serial diodes:
          Only once the pin No of anyone Cathode is identical of another anode.
          two diodes in series is additionally detected as third big diode.
        */
			if (diodes.Cathode[0] == diodes.Anode[1]) {
           diode_sequence = 0x01;	// 0 1
          }
			if (diodes.Anode[0] == diodes.Cathode[1]) {
           diode_sequence = 0x10;	// 1 0
          }
			if (diodes.Cathode[0] == diodes.Anode[2]) {
           diode_sequence = 0x02;	// 0 2
          }
			if (diodes.Anode[0] == diodes.Cathode[2]) {
           diode_sequence = 0x20;	// 2 0
          }
			if (diodes.Cathode[1] == diodes.Anode[2]) {
           diode_sequence = 0x12;	// 1 2
          }
			if (diodes.Anode[1] == diodes.Cathode[2]) {
           diode_sequence = 0x21;	// 2 1
          }
//        if((ptrans.b<3) && (ptrans.c<3)) 
        if(diode_sequence < 0x22) {
           lcd_data('3');
           lcd_MEM_string(Dioden);	//"Diodes "
           SerienDiodenAusgabe();
           goto end3;
        }
     }  // end (NumOfDiodes == 3)
     lcd_MEM_string(Bauteil);		//"Bauteil"
     lcd_MEM_string(Unknown); 		//" unbek."
     lcd_line2(); //2. row 
     lcd_data(NumOfDiodes + '0');
     lcd_data('*');
     lcd_MEM_string(AnKat_str);		//"->|-"
     lcd_MEM_string(Detected);		//" detected"
     goto not_known;
     // end (PartFound == PART_DIODE)
  // ========================================
  } else if (PartFound == PART_TRANSISTOR) {
  // ========================================
#ifdef SEARCH_PARASITIC
    if ((ptrans.count != 0) && (ntrans.count !=0)) {
       // Special Handling of NPNp and PNPn Transistor.
       // If a protection diode is built on the same structur as the NPN-Transistor,
       // a parasitic PNP-Transistor will be detected. 
       ReadCapacity(ntrans.e, ntrans.b);	// read capacity of NPN base-emitter
       n_cval = cap.cval;			// save the found capacity value
       n_cpre  = cap.cpre;			// and dimension
       ReadCapacity(ptrans.b, ptrans.e);	// read capacity of PNP base-emitter
       // check if one hfe is very low. If yes, simulate a very low BE capacity
       if ((ntrans.hfe < 500) && (ptrans.hfe >= 500)) n_cpre = -16; // set NPN BE capacity to low value
       if ((ptrans.hfe < 500) && (ntrans.hfe >= 500)) cap.cpre = -16; // set PNP BE capacity to low value

       if (((n_cpre == cap.cpre) && (cap.cval > n_cval))
					|| (cap.cpre > n_cpre)) {
          // the capacity value or dimension of the PNP B-E is greater than the NPN B-E
          PartMode = PART_MODE_PNP;
       } else {
          PartMode = PART_MODE_NPN;
       }
    }  /* end ((ptrans.count != 0) && (ntrans.count !=0)) */
#endif
    // not possible for mega8, change Pin sequence instead.
		if ((ptrans.count != 0) && (ntrans.count != 0)
				&& (!(RST_PIN_REG & (1 << RST_PIN)))) {
       // if the Start key is still pressed, use the other Transistor
#if 0
       if (PartMode == PART_MODE_NPN) {
          PartMode = PART_MODE_PNP;	// switch to parasitic transistor
       } else {
          PartMode = PART_MODE_NPN;	// switch to parasitic transistor
       }
#else
       PartMode ^= (PART_MODE_PNP - PART_MODE_NPN);
#endif
    }

#ifdef WITH_GRAPHICS
    lcd_set_cursor(0,TEXT_RIGHT_TO_ICON);			// position behind the icon, Line 1
    lcd_big_icon(BJT_NPN|LCD_UPPER_LEFT);	// show the NPN Icon at lower left corner
    if(PartMode == PART_MODE_NPN) {
//       _trans = &ntrans;  is allready selected a default
       lcd_MEM_string(NPN_str);		//"NPN "
       if (ptrans.count != 0) {
          lcd_data('p');		// mark for parasitic PNp
       }
    } else {
       _trans = &ptrans;		// change transistor structure
       lcd_update_icon(bmp_pnp);	// update for PNP
       lcd_MEM_string(PNP_str);		//"PNP "
       if (ntrans.count != 0) {
          lcd_data('n');		// mark for parasitic NPn
       }
    }
#else 	/* only character display */
    if(PartMode == PART_MODE_NPN) {
//       _trans = &ntrans;  is allready selected a default
       lcd_MEM_string(NPN_str);		//"NPN "
       if (ptrans.count != 0) {
          lcd_data('p');		// mark for parasitic PNp
       }
    } else {
       _trans = &ptrans;		// change transistor structure
       lcd_MEM_string(PNP_str);		//"PNP "
       if (ntrans.count != 0) {
          lcd_data('n');		// mark for parasitic NPn
       }
    }
    lcd_space();
#endif

    // show the protection diode of the BJT
    vak_diode_nr = search_vak_diode();
    if (vak_diode_nr < 5) {
    // no side of the diode is connected to the base, this must be the protection diode   
#ifdef WITH_GRAPHICS
       options = 0;
       if (_trans->c != diodes.Anode[vak_diode_nr])
          options |= OPT_VREVERSE;
       lcd_update_icon_opt(bmp_vakdiode,options);	// show the protection diode right to the Icon
#else    /* only character display, show the diode in correct direction */    
       char an_cat;			// diode is anode-cathode type
       an_cat = 0;
 #ifdef EBC_STYLE
  #if EBC_STYLE == 321
       // Layout with 321= style
       an_cat = (((PartMode == PART_MODE_NPN) && (ntrans.c < ntrans.e)) ||
                 ((PartMode != PART_MODE_NPN) && (ptrans.c > ptrans.e)));
  #else
       // Layout with EBC= style
       an_cat = (PartMode == PART_MODE_NPN);
  #endif
 #else
       // Layout with 123= style
       an_cat = (((PartMode == PART_MODE_NPN) && (ntrans.c > ntrans.e))
		|| ((PartMode != PART_MODE_NPN) && (ptrans.c < ptrans.e)));
 #endif
       if (an_cat) {
          lcd_MEM_string(AnKat_str);	//"->|-"
       } else {
          lcd_MEM_string(KatAn_str);	//"-|<-"
       }
#endif    /* !WITH_GRAPHICS */
    }  /* endif vak_diode_nr < 6 */

#ifdef WITH_GRAPHICS
    lcd_draw_trans_pins(-7, 16);	// show the pin numbers
    lcd_next_line(TEXT_RIGHT_TO_ICON);	// position behind the icon, Line 2
    lcd_MEM_string(hfe_str);		//"B="  (hFE)
    DisplayValue(_trans->hfe,-2,0,3);

    lcd_next_line(TEXT_RIGHT_TO_ICON+1-LOW_H_SPACE); // position behind the icon+1, Line 3
    lcd_data('I');
    if (_trans->current >= 10000) {
       lcd_data('e');				// emitter current has 10mA offset
       _trans->current -= 10000;
    } else {
       lcd_data('c');
    }
    lcd_equal();			// lcd_data('=');
    DisplayValue16(_trans->current,-6,'A',2);	// display Ic or Ie current

    lcd_next_line(TEXT_RIGHT_TO_ICON); // position behind the icon, Line 4
    lcd_MEM_string(Ube_str);		//"Ube="
    Display_mV(_trans->uBE,3-LOW_H_SPACE);
    last_line_used = 1;

 #ifdef SHOW_ICE
    if (_trans->ice0 > 0) {
       lcd_next_line_wait(TEXT_RIGHT_TO_ICON-1-LOW_H_SPACE); // position behind the icon, Line 4 & wait and clear last line
       lcd_MEM2_string(ICE0_str);		// "ICE0="
       DisplayValue16(_trans->ice0,-6,'A',2);	// display ICEO
    }
    if (_trans->ices > 0) {
       lcd_next_line_wait(TEXT_RIGHT_TO_ICON-1-LOW_H_SPACE); // position behind the icon, Line 4 & wait and clear last line
       lcd_MEM2_string(ICEs_str);		// "ICEs="
       DisplayValue16(_trans->ices,-6,'A',2);	// display ICEs
    }
 #endif
#else		/* character display */
    PinLayout('E','B','C'); 		//  EBC= or 123=...
    lcd_line2(); //2. row 
    lcd_MEM_string(hfe_str);		//"B="  (hFE)
    DisplayValue(_trans->hfe,-2,0,3);
 #if FLASHEND > 0x1fff
    lcd_space();

    lcd_data('I');
    if (_trans->current >= 10000) {
       lcd_data('e');				// emitter current has 10mA offset
       _trans->current -= 10000;
    } else {
       lcd_data('c');
    }
    lcd_equal();			// lcd_data('=');
    DisplayValue16(_trans->current,-6,'A',2);	// display Ic or Ie current
 #endif

 #if defined(SHOW_ICE)
    lcd_next_line_wait(0);		// next line, wait 5s and clear line 2
    lcd_MEM_string(Ube_str);		//"Ube=" 
    Display_mV(_trans->uBE,3);

    if (_trans->ice0 > 0) {
       lcd_next_line_wait(0);		// next line, wait 5s and clear line 2
       lcd_MEM2_string(ICE0_str);		// "ICE0="
       DisplayValue16(_trans->ice0,-6,'A',3);
    }
    if (_trans->ices > 0) {
       lcd_next_line_wait(0);		// next line, wait 5s and clear line 2
       lcd_MEM2_string(ICEs_str);		// "ICEs="
       DisplayValue16(_trans->ices,-6,'A',3);
    }
 #endif
#endif  /* WITH_GRAPHICS */
#ifdef SHOW_VAKDIODE
    if (vak_diode_nr < 5) {
       lcd_next_line_wait(0); 		// next line, wait 5s and clear line 2/4
       DiodeSymbol_withPins(vak_diode_nr);
       lcd_MEM_string(Uf_str);			//"Uf="
       mVAusgabe(vak_diode_nr);
       uart_newline();			// MAURO not verified ('D')
    } /* end if (vak_diode_nr < 5) */
#endif
#ifdef WITH_GRAPHICS
    PinLayoutLine('E','B','C'); 		//  Pin 1=E ...
    uart_newline();			// MAURO OK BJT ('E')
#endif
    goto tt_end;
    // end (PartFound == PART_TRANSISTOR)

  // ========================================
  } else if (PartFound == PART_FET) {	/* JFET or MOSFET */
  // ========================================
#ifdef WITH_GRAPHICS
    unsigned char fetidx = 0;
    lcd_set_cursor(0,TEXT_RIGHT_TO_ICON);	// position behind the icon, Line 1
#endif
    if((PartMode&P_CHANNEL) == P_CHANNEL) {
       lcd_data('P');			//P-channel
       _trans = &ptrans;
#ifdef WITH_GRAPHICS
       fetidx = 2;
#endif
    } else {
       lcd_data('N');			//N-channel
//       _trans = &ntrans;	is allready selected as default
    }
    lcd_data('-');		// minus is used for JFET, D-MOS, E-MOS ...

    uint8_t part_code;
    part_code = PartMode&0x0f;
#ifdef WITH_GRAPHICS
    if (part_code == PART_MODE_JFET) {
       lcd_MEM_string(jfet_str);	//"JFET"
       lcd_big_icon(N_JFET|LCD_UPPER_LEFT);
       if (fetidx != 0) {
          lcd_update_icon(bmp_p_jfet); // update the n_jfet bitmap to p_jfet
       }
    } else {		// no JFET
       if ((PartMode&D_MODE) == D_MODE) {
          lcd_data('D');			// N-D or P-D
          fetidx += 1;
       } else {
          lcd_data('E');			// N-E or P-E
       }
       if (part_code == (PART_MODE_IGBT)) {
          lcd_MEM_string(igbt_str);	//"-IGBT"
          lcd_big_icon(N_E_IGBT|LCD_UPPER_LEFT);
          if (fetidx == 1)  lcd_update_icon(bmp_n_d_igbt);
          if (fetidx == 2)  lcd_update_icon(bmp_p_e_igbt);
          if (fetidx == 3)  lcd_update_icon(bmp_p_d_igbt);
       } else {
          lcd_MEM_string(mosfet_str);	//"-MOS "
          lcd_big_icon(N_E_MOS|LCD_UPPER_LEFT);
          if (fetidx == 1)  lcd_update_icon(bmp_n_d_mos);
          if (fetidx == 2)  lcd_update_icon(bmp_p_e_mos);
          if (fetidx == 3)  lcd_update_icon(bmp_p_d_mos);
       }
    } /* end PART_MODE_JFET */
#else	/* normal character display */
    if (part_code == PART_MODE_JFET) {
       lcd_MEM_string(jfet_str);	//"-JFET"
    } else {		// no JFET
       if ((PartMode&D_MODE) == D_MODE) {
          lcd_data('D');			// N-D or P-D
       } else {
          lcd_data('E');			// N-E or P-E
       }
       if (part_code == (PART_MODE_IGBT)) {
          lcd_MEM_string(igbt_str);	//"-IGBT"
       } else {
          lcd_MEM_string(mosfet_str);	//"-MOS "
       }
    } /* end PART_MODE_JFET */

    if (part_code == PART_MODE_IGBT) {
       PinLayout('E','G','C'); 		//  SGD= or 123=...
    } else if (part_code == PART_MODE_JFET) {
       PinLayout('?','G','?'); 		//  ?G?= or 123=...
    } else {
       PinLayout('S','G','D'); 		//  SGD= or 123=...
    }
#endif  /* WITH_GRAPHICS */

    vak_diode_nr = search_vak_diode();
    if(vak_diode_nr < 5) {
       //MOSFET with protection diode; only with enhancement-FETs

#ifndef WITH_GRAPHICS
 #if FLASHEND <= 0x1fff
       char an_cat;			// diode is anode-cathode type
       an_cat = 0;
  #ifdef EBC_STYLE
   #if EBC_STYLE == 321
       // layout with 321= style
       an_cat = (((PartMode&P_CHANNEL) && (ptrans.c > ptrans.e)) || ((!(PartMode&P_CHANNEL)) && (ntrans.c < ntrans.e)));
   #else
       // Layout with SGD= style
       an_cat = (PartMode&P_CHANNEL);	/* N or P MOS */
   #endif
  #else /* EBC_STYLE not defined */
       // layout with 123= style
			an_cat = (((PartMode & P_CHANNEL) && (ptrans.c < ptrans.e))
					|| ((!(PartMode & P_CHANNEL)) && (ntrans.c > ntrans.e)));
  #endif /* end ifdef EBC_STYLE */
       //  show diode symbol in right direction  (short form for less flash memory)
       if (an_cat) {
          lcd_data(LCD_CHAR_DIODE1);	//show Diode symbol >|
       } else {
          lcd_data(LCD_CHAR_DIODE2);	//show Diode symbol |<
       }
 #endif
#endif  /* not WITH_GRAPHICS */
#ifdef WITH_GRAPHICS
       options = 0;
       if (_trans->c != diodes.Anode[vak_diode_nr])
          options |= OPT_VREVERSE;
       lcd_update_icon_opt(bmp_vakdiode,options);	// update Icon with protection diode
#endif

    } /* end if NumOfDiodes == 1 */

#ifdef WITH_GRAPHICS
    lcd_draw_trans_pins(-7, 16);	// update of pin numbers must be done after diode update
    lcd_next_line(TEXT_RIGHT_TO_ICON);	// position text behind the icon, Line 2
 #if LCD_LINES > 6
       lcd_next_line(TEXT_RIGHT_TO_ICON);	// double line
 #endif
    if((PartMode&D_MODE) != D_MODE) {	//enhancement-MOSFET
       lcd_MEM_string(vt_str+1);		// "Vt="
       Display_mV(_trans->gthvoltage,2);	//Gate-threshold voltage
	//Gate capacity
       ReadCapacity(_trans->b,_trans->e);	//measure capacity
       lcd_next_line(TEXT_RIGHT_TO_ICON);	// position text behind the icon, Line 3
       lcd_show_Cg();	// show Cg=xxxpF
 #ifdef SHOW_R_DS
       lcd_show_rds(TEXT_RIGHT_TO_ICON-1); 	// show RDS at column behind the icon -1
 #endif
    } else {   /* depletion mode */
       if ((PartMode&0x0f)  != PART_MODE_JFET) {     /* kein JFET */
          ReadCapacity(_trans->b,_trans->e);	//measure capacity
          lcd_show_Cg();	// show Cg=xxxpF
  #ifdef FET_Idss
       } else { 	// it is a JFET
          // display the I_DSS, if measured
          if (_trans->uBE!=0) {
             static const unsigned char str_Idss[] MEM_TEXT = "Idss=";
             lcd_MEM_string(str_Idss);
             DisplayValue16(_trans->uBE,-6,'A',2);
          }
  #endif
       }
       // set cursor below the icon
  #define LINE_BELOW_ICON ((ICON_HEIGHT/8)/((FONT_HEIGHT+7)/8))
 #if LCD_LINES > 6
       lcd_set_cursor((LINE_BELOW_ICON + 1) * PAGES_PER_LINE,0);
 #else
       lcd_set_cursor(LINE_BELOW_ICON * PAGES_PER_LINE,0);
 #endif
       lcd_data('I');
 #if (LCD_LINE_LENGTH > 17)
       lcd_data('d');
 #endif
       lcd_equal();			// lcd_data('=');
       DisplayValue16(_trans->current,-6,'A',2);
       lcd_MEM_string(Vgs_str);		// "@Vg="
       Display_mV(_trans->gthvoltage,2);	//Gate-threshold voltage

 #ifdef SHOW_ICE
       // Display also the cutoff gate voltage, idea from Pieter-Tjerk
       if (_trans->ice0<4800) { // can't trust cutoff voltage if close to 5V supply voltage, since then the transistor may not have been cut off at all
          lcd_next_line_wait(0);
          lcd_data('I');
  #if (LCD_LINE_LENGTH > 17)
          lcd_data('d');
  #endif
          lcd_equal();			// lcd_data('=');
          DisplayValue16(0,-5,'A',2);
          lcd_MEM_string(Vgs_str);		// "@Vg="
          Display_mV(_trans->ice0,2);	// cutoff Gate voltage
 #endif
       }
 #ifdef SHOW_R_DS
       lcd_show_rds(0);                // show Drain-Source resistance RDS at column 0
 #endif
    }	/* end of enhancement or depletion mode WITH_GRAPHICS */
#else	/* character display */
    if((PartMode&D_MODE) != D_MODE) {	//enhancement-MOSFET
	//Gate capacity
       lcd_line2();		// line 2
       ReadCapacity(_trans->b,_trans->e);	//measure capacity
       lcd_show_Cg();	// show Cg=xxxpF
       lcd_MEM_string(vt_str);		// " Vt="
       Display_mV(_trans->gthvoltage,2);	//Gate-threshold voltage
  #ifdef SHOW_R_DS
       lcd_show_rds(0);                // show Drain-Source resistance RDS at column 0
  #endif
    } else {
      // depletion
 #if FLASHEND > 0x1fff
       if ((PartMode&0x0f)  != PART_MODE_JFET) {     /* kein JFET */
          lcd_next_line(0);		// line 2
          ReadCapacity(_trans->b,_trans->e);	//measure capacity
          lcd_show_Cg();	// show Cg=xxxpF
  #ifdef FET_Idss
       } else {     // it is a JFET
          // display the I_DSS, if measured
          if (_trans->uBE!=0) {
             lcd_next_line(0);
             static const unsigned char str_Idss[] MEM_TEXT = "Idss=";
             lcd_MEM_string(str_Idss);
             DisplayValue16(_trans->uBE,-6,'A',2);
          }
  #endif
       }
       lcd_next_line_wait(0);		// line 2 or 3, if possible & wait and clear last line
 #endif
       lcd_data('I');			// show I=xmA@Vg=y.yV at line 2 or 3
 #if (LCD_LINE_LENGTH > 17)
       lcd_data('d');
 #endif
       lcd_equal();			// lcd_data('=');
       DisplayValue16(_trans->current,-6,'A',2);
       lcd_MEM_string(Vgs_str);		// "@Vg="
       Display_mV(_trans->gthvoltage,2);	//Gate-threshold voltage
 #ifdef SHOW_ICE
       // Display also the cutoff gate voltage, idea from Pieter-Tjerk
       if (_trans->ice0<4800) { // can't trust cutoff voltage if close to 5V supply voltage, since then the transistor may not have been cut off at all
          lcd_next_line_wait(0);
          lcd_data('I');
  #if (LCD_LINE_LENGTH > 17)
          lcd_data('d');
  #endif
          lcd_equal();			// lcd_data('=');
          DisplayValue16(0,-5,'A',2);
          lcd_MEM_string(Vgs_str);		// "@Vg="
          Display_mV(_trans->ice0,2);	// cutoff Gate voltage
       }
 #endif
 #ifdef SHOW_R_DS
       lcd_show_rds(0);                // show Drain-Source resistance RDS at column 0
 #endif
    }   /* end of enhancement or depletion mode */
#endif  /* WITH_GRAPHICS or without */

#if DebugOut == 5
    lcd_line4();
    lcd_data('>');
    lcd_data('|');
    lcd_space();
    DisplayValue16(NumOfDiodes,0,' ',3);
#endif
#if FLASHEND > 0x1fff
    if (part_code != PART_MODE_JFET) {
       for (ii=0;ii<NumOfDiodes;ii++) {
          // there is enough space for long form of presenting protection diode
 #if LCD_LINES > 6
          if (ii == 0) lcd_next_line(0);		// line 5 , if possible
 #endif
          lcd_next_line_wait(0);		// line 4, if possible & wait 5s and clear last line 
          DiodeSymbol_withPins(ii);
          lcd_MEM_string(Uf_str);			//"Uf="
          mVAusgabe(ii);
//          uart_newline();			// MAURO OK N-E-MOS/IGBT ('F')
       } /* end for ii (NumOfDiodes) */
    }  /* PART_MODE != JFET */
#endif
#ifdef WITH_GRAPHICS
    if (part_code == PART_MODE_IGBT) {
       PinLayoutLine('E','G','C'); 		//  Pin 1=...
    } else if (part_code == PART_MODE_JFET) {
       PinLayoutLine('?','G','?'); 		//  Pin 1=...
    } else {
       PinLayoutLine('S','G','D'); 		//  Pin 1=...
    }
       uart_newline();			// MAURO OK IGBT ('G')
#endif
    goto tt_end;
  }  /* end (PartFound == PART_FET) */

//   if(PartFound == PART_RESISTOR) 
resistor_out:
  if (ResistorsFound != 0) {
    if (ResistorsFound == 1) { // single resistor

       rpins.pw = Rnum2pins(ResistorList[0]);	// get pin numbers for resistor 1
#if FLASHEND <= 0x1fff
       lcd_testpin(rpins.pb[0]);  	//Pin-number 1
       lcd_MEM_string(Resistor_str);	// -[=]-
       lcd_testpin(rpins.pb[1]);	//Pin-number 2
       lcd_line2(); //2. row 
       RvalOut(ResistorList[0]);
#else
 #if FLASHEND > 0x3fff
       if ((ResistorList[0] == 1) && (NumOfDiodes == 0)) {
          // is the TP1:TP3 resistor and no additional diode
          show_Resis13();		// call of the special resistor measurement
          goto shut_off;		// key is pressed or timeout
       }
 #endif
       show_resis(rpins.pb[0],rpins.pb[1],0);
#endif

    } else { // multiple resistors found, R-Max suchen

       ii = ResistorList[0];	// first resistor in the list with number 0,1,2
       if (ResistorVal[ResistorList[1]] > ResistorVal[ii])
          ii = ResistorList[1]; // second resistor in the list with number 0,1,2
       if (ResistorsFound == 2) {
          ii = (3 - ResistorList[0] - ResistorList[1]);
       } else {
          if (ResistorVal[ResistorList[2]] > ResistorVal[ii]) {
             ii = ResistorList[2];
          }
       }
       // ResistorVal[0] TP1:TP2, [1] TP1:TP3, [2] TP2:TP3
#if (TP2!=TP1+1 || TP3!=TP2+1)
       x = TP1;
       y = TP3;
       z = TP2;
       if (ii == 1) {	/* TP1:TP3 is maximum */
          x = TP1;
          y = TP2;
          z = TP3;
       }
       if (ii == 2) {	/* TP2:TP3 is maximum */
          x = TP2;
          y = TP1;
          z = TP3;
       }
#else
    // the following does the same as the above, but more cryptically (and in fewer instructions)
       x = TP1+(ii>>1);
       y = TP3-ii;
       z = TP2+(ii>0);
#endif
       lcd_testpin(x);  	//Pin-number 1
       lcd_MEM_string(Resistor_str);    // -[=]-
       lcd_testpin(y);		//Pin-number 2
       lcd_MEM_string(Resistor_str);    // -[=]-
       lcd_testpin(z);		//Pin-number 3

       lcd_next_line(0);
       // output resistor values in right order
    //  if (ii == 0) { /* resistor 0 has maximum */
    //     RvalOut(1);
    //     RvalOut(2);
    //  }
    //  if (ii == 1) { /* resistor 1 has maximum */
    //     RvalOut(0);
    //     RvalOut(2);
    //  }
    //  if (ii == 2) { /* resistor 2 has maximum */
    //     RvalOut(0);
    //     RvalOut(1);
    //  }
    // again a shorter but cryptical equivalent:
       RvalOut(ii==0);
       RvalOut(2-(ii>>1));
    }  // end ResistorsFound==1
    goto tt_end;

  } // end (PartFound == PART_RESISTOR)

  lcd_MEM_string(TestFailed1); 	//"Kein,unbek. oder"
  lcd_line2(); //2. row 
#if defined(LANG_ITALIAN) || defined(LANG_FRANCAIS)	//italiano or francais
  lcd_MEM_string(Bauteil);		//"campione"
  lcd_space();
  lcd_MEM_string(TestFailed2); 		//"guasto "
#else
  lcd_MEM_string(TestFailed2); 		//"defektes "
  lcd_MEM_string(Bauteil);		//"Bauteil"
#endif
not_known:
//   uart_newline();		// MAURO added
#ifdef WITH_GRAPHICS
     lcd_big_icon(QUESTION);		// show big question mark
#endif
#if POWER_OFF+0 > 1
  empty_count++;
  mess_count = 0;
#endif
  max_time = SHORT_WAIT_TIME;		// use allways the short wait time
  goto end2;

//gakAusgabe:
//  PinLayout(Cathode_char,'G','A'); 	// CGA= or 123=...
TyUfAusgabe:
#ifdef WITH_THYRISTOR_GATE_V
 #ifdef WITH_GRAPHICS
  lcd_set_cursor(1*PAGES_PER_LINE,TEXT_RIGHT_TO_ICON);		// position behind the icon,line 2
 #else
  lcd_line2(); //2. row 
 #endif
  lcd_MEM_string(Uf_str);		// "Uf="
 #ifdef WITH_PUT
  Display_mV(_trans->uBE,2);    // needed instead of ntrans for PUT, but costs 4 bytes more flash...
 #else
  Display_mV(ntrans.uBE,2);
 #endif
//  uart_newline();		// MAURO ('I')
#endif
//- - - - - - - - - - - - - - - - - - - - - - - - - - - - 
 tt_end:
#if POWER_OFF+0 > 1
  empty_count = 0;		// reset counter, if part is found
  mess_count++;			// count measurements
#endif
  max_time = display_time;	// full specified wait time

 end2:
  ADC_DDR = (1<<TPRELAY) | TXD_MSK; 	// switch pin with reference to GND, release relay
  lcd_refresh();			// write the pixels to display, ST7920 only
	while (!(RST_PIN_REG & (1 << RST_PIN)))
		;	//wait ,until button is released
#ifdef WITH_ROTARY_SWITCH
wait_again:
#endif
  ii = wait_for_key_ms(max_time);
#if POWER_OFF+0 > 1
 #ifdef WITH_ROTARY_SWITCH
  if ((ii > 0) || (rotary.incre > 0))
 #else
  if (ii > 0)
 #endif
  {
     empty_count = 0;		// reset counter, if any switch activity
     mess_count = 0; }
#endif
#ifdef WITH_MENU
 #ifdef WITH_ROTARY_SWITCH
  if ((ii >=50) || (rotary.incre > 2))
 #else
  if (ii >= 50) 
 #endif
  {
     // menu selected by long key press or rotary switch
		while(function_menu());// start the function menu
     goto loop_start;
  }
#endif
	if (ii != 0)
		goto loop_start;
	// key is pressed again, repeat measurement
#ifdef WITH_ROTARY_SWITCH
  if (rotary.incre > 0) goto wait_again;
#endif
#ifdef POWER_OFF
 #if POWER_OFF > 127
  #define POWER2_OFF 255
 #else
  #define POWER2_OFF POWER_OFF*2
 #endif
 #if POWER_OFF+0 > 1
  if ((empty_count < POWER_OFF) && (mess_count < POWER2_OFF)) {
     goto loop_start;			// repeat measurement POWER_OFF times
  }
 #endif
  // only one Measurement requested, shut off
 #if FLASHEND > 0x3fff
shut_off:
  // look, if the tester is uncalibrated (C-source will be included directly)
  lcd_cursor_off();
  /* HelpCalibration Begin */
#if defined(AUTO_CAL) && (FLASHEND > 0x3fff)
// Check is direct included in the main source of the TransistorTester
// a function with a call from main will use additional 38 bytes of flash
  // define additional variables , ii is already defined in main
 #define TIME_TO_READ 10000
  if (UnCalibrated) {
#ifndef SHORT_UNCAL_MSG
  unsigned int jj;
  char zeich;
  uint8_t space_pos;
  uint8_t line_nr;
  uint8_t sub_line;
    // Output the help text for calibration.
    // The text is formatted for two 16 character display lines.
    jj = 0;
    zeich = ' ';		// initial value for while loop
    line_nr = LCD_LINES;		// begin with the first LCD line, but don't wait
    while (zeich != (char)0) {	// zero is end of text
       space_pos = LCD_LINE_LENGTH;		// if no space is found
       for (ii=0;ii<(LCD_LINE_LENGTH+1);ii++) {	// look for the last space character
         zeich = pgm_read_byte(&HelpCalibration_str[ii+jj]);
         if (zeich == ' ')  space_pos = ii; // save the position
       }
       if (line_nr == 0) {
          // it is the first LCD line, wait for showing the last message
          if ((wait_for_key_ms(TIME_TO_READ)) != 0) break;	// key pressed
       }
       sub_line = line_nr % LCD_LINES;
       if (sub_line == 0) lcd_clear();  // clear display, line_nr is 0 or 4
       lcd_set_cursor(sub_line*PAGES_PER_LINE ,0);
       uart_newline();
       wdt_reset();
       line_nr = (line_nr + 1) % LCD_LINES;
       for (ii=0;ii<space_pos;ii++) {
         zeich = pgm_read_byte(&HelpCalibration_str[ii+jj]);
         if (zeich == (char)0) break;	// end of text found
         if (zeich == LCD_CHAR_INSEP) {
            lcd_space();	// replace with space
         } else {
            lcd_data(zeich);		// display the character without offset
         }
       }
       if (zeich == (char)0) break;	// end of text found
       jj += space_pos;		// start position of line 2
       if((pgm_read_byte(&HelpCalibration_str[jj])) == ' ') jj++; // no space at begin of line
    }  /* end while */
#else
    lcd_clear();
    lcd_pgm_string(HelpCalibration_str);	// only short message!
	#ifdef WITH_UART		//Mauro 
//		uart_newline();		//Mauro 
	#endif					//Mauro 
#endif
    wait_for_key_ms(TIME_TO_READ);	// key pressed
  }
#endif  /* AUTO_CAL && (FLASHEND > 0x3fff) */
/* HelpCalibration End */

 #endif
//  MCUSR = 0;
  switch_tester_off();
#else   /* no POWER_OFF */
shut_off:
  // look, if the tester is uncalibrated (C-source will be included directly)
  lcd_cursor_off();
  /* HelpCalibration Begin */
#if defined(AUTO_CAL) && (FLASHEND > 0x3fff)
// Check is direct included in the main source of the TransistorTester
// a function with a call from main will use additional 38 bytes of flash
  // define additional variables , ii is already defined in main
 #define TIME_TO_READ 10000
  if (UnCalibrated) {
#ifndef SHORT_UNCAL_MSG
  unsigned int jj;
  char zeich;
  uint8_t space_pos;
  uint8_t line_nr;
  uint8_t sub_line;
    // Output the help text for calibration.
    // The text is formatted for two 16 character display lines.
    jj = 0;
    zeich = ' ';		// initial value for while loop
    line_nr = LCD_LINES;		// begin with the first LCD line, but don't wait
    while (zeich != (char)0) {	// zero is end of text
       space_pos = LCD_LINE_LENGTH;		// if no space is found
       for (ii=0;ii<(LCD_LINE_LENGTH+1);ii++) {	// look for the last space character
         zeich = pgm_read_byte(&HelpCalibration_str[ii+jj]);
         if (zeich == ' ')  space_pos = ii; // save the position
       }
       if (line_nr == 0) {
          // it is the first LCD line, wait for showing the last message
          if ((wait_for_key_ms(TIME_TO_READ)) != 0) break;	// key pressed
       }
       sub_line = line_nr % LCD_LINES;
       if (sub_line == 0) lcd_clear();  // clear display, line_nr is 0 or 4
       lcd_set_cursor(sub_line*PAGES_PER_LINE ,0);
       uart_newline();
       wdt_reset();
       line_nr = (line_nr + 1) % LCD_LINES;
       for (ii=0;ii<space_pos;ii++) {
         zeich = pgm_read_byte(&HelpCalibration_str[ii+jj]);
         if (zeich == (char)0) break;	// end of text found
         if (zeich == LCD_CHAR_INSEP) {
            lcd_space();	// replace with space
         } else {
            lcd_data(zeich);		// display the character without offset
         }
       }
       if (zeich == (char)0) break;	// end of text found
       jj += space_pos;		// start position of line 2
       if((pgm_read_byte(&HelpCalibration_str[jj])) == ' ') jj++; // no space at begin of line
    }  /* end while */
#else
    lcd_clear();
    lcd_pgm_string(HelpCalibration_str);	// only short message!
	#ifdef WITH_UART		//Mauro 
//		uart_newline();		//Mauro 
	#endif					//Mauro 
#endif
    wait_for_key_ms(TIME_TO_READ);	// key pressed
  }
#endif  /* AUTO_CAL && (FLASHEND > 0x3fff) */
/* HelpCalibration End */

#endif
	goto loop_start;
	// POWER_OFF not selected, repeat measurement
//  return 0;

end3:
//        uart_newline();		// MAURO  ('H')
  // the diode  is already shown on the LCD
	if (ResistorsFound == 0)
		goto tt_end;
  ADC_DDR = (1<<TPRELAY) | TXD_MSK; 	// switch pin with reference to GND, release relay
  // there is one resistor or more detected
#if (LCD_LINES > 3)
  ADC_DDR =  TXD_MSK; 	// switch pin with reference to input, activate relay
 #ifdef WAIT_LINE2_CLEAR
  lcd_next_line_wait(0);
 #else
  lcd_line3();
 #endif
#else
  wait_for_key_ms(display_time);
  ADC_DDR =  TXD_MSK; 	// switch pin with reference to input, activate relay
  lcd_clear();
//#if FLASHEND > 0x1fff
  lcd_data('0'+NumOfDiodes);
  lcd_MEM_string(Dioden);	//"Diodes "
//#endif
#endif
  goto resistor_out;

}   // ****** end setup()  ***** end main




// function search_vak_diode try to find a diode, which has no side connected to the transistor base
// returns 20, if no diode found
uint8_t search_vak_diode() {
    uint8_t ii;
    for (ii=0; ii<NumOfDiodes; ii++) {
			if ((diodes.Anode[ii] == _trans->b)
					|| (diodes.Cathode[ii] == _trans->b))
				continue;
       // no side of the diode is connected to the base, this must be the protection diode   
       if (diodes.Voltage[ii] > 1000) break; // Voltage is too high for protection diode
       return ii;
    }
    return 20;
}

/* init_parts initialize all parts to nothing found */
void init_parts(void) {
  PartFound = PART_NONE;	// no part found
  NumOfDiodes = 0;		// Number of diodes = 0
  ptrans.count = 0;		// Number of found P type transistors
  ntrans.count = 0;		// Number of found N type transistors
  PartMode = PART_MODE_NONE;
  WithReference = 0;		// no precision reference voltage
  ResistorsFound = 0;		// no resistors found
  ResistorChecked[0] = 0;
  ResistorChecked[1] = 0;
  ResistorChecked[2] = 0;
  cap.ca = TP1;
  cap.cb = TP3;
#if FLASHEND > 0x1fff
  inductor_lpre = 0;		// mark as zero
  cap.v_loss = 0;		// set Vloss to zero
#endif
  cap.cval_max = 0;		// set max to zero
  cap.cpre_max = -15;	// set max to fF unit
}

void switch_tester_off(void)
{
 #ifdef WITH_UART
  uart_putc('O');	// MAURO
  uart_putc('F');	// MAURO
  uart_putc('F');	// MAURO
//  uart_newline();	// MAURO
 #endif
 #if ((LCD_ST_TYPE == 7565) || (LCD_ST_TYPE == 1306))
  lcd_powersave();			// set graphical display to power save mode
 #endif
  ON_PORT &= ~(1<<ON_PIN);		//switch off power
  wait2s();
  wait_for_key_ms(0); //never ending loop 
}


#ifdef WITH_SELFTEST
 /* Begin AutoCheck.c */
// Selftest of the device and calibration 
#ifdef WITH_SELFTEST
void AutoCheck(uint8_t test_mode) {
  /* (test_mode & 0x0f) == 0 , only calibration without T1-T7 */
  /* (test_mode & 0x0f) == 1 , calibration and additional T1-T7 */
  /* (test_mode & 0xf0) == 0 , check for shorted probes, if unshorted, return */
  /* (test_mode & 0xf0) == 0x10 , ask for shorted probes */

 uint8_t ww;		// counter for repeating the tests
 int  adcmv[7];
 #ifdef EXTENDED_TESTS
  uint16_t u680;	// 3 * (Voltage at 680 Ohm)
  uint8_t taste;	// ist key pressed? 0 = no
 #else
  #ifndef NO_TEST_T1_T7
  #warning "Selftest without extended tests T1 to T7!"
  #endif
 #endif
 #if defined(EXTENDED_TESTS) || defined(WITH_MENU)
  uint8_t tt;		// number of running test
 #endif

 #ifdef FREQUENCY_50HZ
  uint8_t ff50;		// loop counter for 2s
 #endif

// define the maximum count of repetitions MAX_REP
#define MAX_REP 4

 #ifdef AUTO_CAL
uint8_t cap_found;	// counter for found capacitor
  #ifdef AUTOSCALE_ADC
int8_t udiff;		// difference between ADC Voltage with VCC or Bandgap reference
int8_t udiff2;
  #endif
 #endif
PartFound = PART_NONE;		// no part found before
if ((test_mode & 0xf0) == 0) {
  // probed should be shorted already to begin selftest
  if (AllProbesShorted() != 3) return;
  lcd_clear();
  lcd_MEM_string(SELFTEST);		// "Selftest mode.."
  lcd_line2();
  lcd_data('?');			// wait for key pressed for confirmation
  if (wait_for_key_ms(2000) > 10) goto begin_selftest;	// key is pressed again
 #ifdef WITH_MENU
} else {
  // report to user, that probes should be shorted
  ww = 0;
  for (tt=0;tt<150;tt++) {	/* wait about 30 seconds for shorted probes */
    lcd_clear();
    lcd_MEM2_string(SHORT_PROBES_str);	// message "Short probes!" to LCD
    if (AllProbesShorted() == 3) {
       ww++;	// all probes now shorted
    } else {
       ww = 0;	// connection not stable, retry
    }
    if (ww > 3) break;	// connection seems to be stable
    lcd_refresh();		// write the pixels to display, ST7920 only
    wait_about200ms();			// wait 200ms and try again
  }  /* end for (tt...) */
  if (tt < 150) goto begin_selftest;		// is shorted before time limit
  goto no_zero_resistance;			// skip measuring of the zero resistance
 #endif
}
// no key pressed for 2s
lcd_clear();
lcd_MEM_string(VERSION_str);	//"Version ..."
return;

begin_selftest:
lcd_line2();
Calibrate_UR();		// get Reference voltage, Pin resistance
lcd_MEM2_string(R0_str);		// "R0="
eeprom_write_byte((uint8_t *)(&EE_ESR_ZEROtab[2]), (uint8_t)0);	// clear zero offset
eeprom_write_byte((uint8_t *)(&EE_ESR_ZEROtab[3]), (uint8_t)0);	// clear zero offset
eeprom_write_byte((uint8_t *)(&EE_ESR_ZEROtab[1]), (uint8_t)0);	// clear zero offset

adcmv[0] = GetESR(TP3, TP1);
adcmv[1] = GetESR(TP3, TP2);
adcmv[2] = GetESR(TP2, TP1);
DisplayValue16(adcmv[0],-2,' ',3);
DisplayValue16(adcmv[1],-2,' ',3);
DisplayValue16(adcmv[2],-2,LCD_CHAR_OMEGA,3);
if (adcmv[0] >= 90) {
  adcmv[0] = ESR_ZERO;	// set back to default value
}
eeprom_write_byte((uint8_t *)(&EE_ESR_ZEROtab[2]), (uint8_t)adcmv[0]);	// fix zero offset
if (adcmv[1] >= 90) {
  adcmv[1] = ESR_ZERO;	// set back to default value
}
eeprom_write_byte((uint8_t *)(&EE_ESR_ZEROtab[3]), (uint8_t)adcmv[1]);	// fix zero offset
if (adcmv[2] >= 90) {
  adcmv[2] = ESR_ZERO;	// set back to default value
}
eeprom_write_byte((uint8_t *)(&EE_ESR_ZEROtab[1]), (uint8_t)adcmv[2]);	// fix zero offset
last_line_used = 2;
wait_for_key_5s_line2();		// wait up to 5 seconds and clear line 2

 #ifdef WITH_MENU
no_zero_resistance:
 #endif
 #ifdef EXTENDED_TESTS
#define TEST_COUNT 8
if((test_mode & 0x0f) == 1) {  /* full test requested */

for(tt=1;tt<TEST_COUNT;tt++) {		// loop for all Tests
for(ww=0;ww<MAX_REP;ww++) {	// repeat the test MAX_REP times
   lcd_clear_line2();		// clear total line 2
   lcd_clear_line1();		// clear total line 1
   lcd_data('T');			//output the Testmode "T"
   u2lcd(tt);		//lcd_string(utoa(tt, outval, 10));	//output Test number
   lcd_space();
				//############################################
   if (tt == 1) {   // output of reference voltage and factors for capacity measurement
      lcd_MEM2_string(URef_str);	//"URef="
      Display_mV(ref_mv,4);
      lcd_line2();			//Cursor to column 1, row 2
      lcd_MEM2_string(RHfakt_str);	//"RHf="
      u2lcd(RHmultip);	//lcd_string(utoa(RHmultip, outval, 10));
      ADCconfig.Samples = R_ANZ_MESS;	// set number of ADC reads near to maximum
   }
				//############################################
   if (tt == 2) { // how equal are the RL resistors? 
      u680 = ((long)ADCconfig.U_AVCC * (PIN_RM + R_L_VAL) / (PIN_RM + R_L_VAL + R_L_VAL + PIN_RP));
      R_PORT = 1<<PIN_RL1;		//RL1 to VCC
      R_DDR = (1<<PIN_RL1) | (1<<PIN_RL2);	//RL2 to -
      adcmv[0] = W20msReadADC(TP1);
      adcmv[0] -= u680;
      R_DDR = (1<<PIN_RL1) | (1<<PIN_RL3);	//RL3 to -
      adcmv[1] = W20msReadADC(TP1);
      adcmv[1] -= u680;
      R_PORT = 1<<PIN_RL2;		//RL2 to VCC
      R_DDR = (1<<PIN_RL2) | (1<<PIN_RL3);	//RL3 to -
      adcmv[2] = W20msReadADC(TP2);
      adcmv[2] -= u680;
      lcd_MEM_string(RLRL_str);	// "RLRL"
   }
				//############################################
   if (tt == 3) { // how equal are the RH resistors
      R_PORT = 1<<PIN_RH1;		//RH1 to VCC
      R_DDR = (1<<PIN_RH1) | (1<<PIN_RH2);	//RH2 to -
      adcmv[0] = W20msReadADC(TP1);
      adcmv[3] = ADCconfig.U_AVCC / 2;
      adcmv[0] -= adcmv[3];
      R_DDR = (1<<PIN_RH1) | (1<<PIN_RH3);	//RH3 to -
      adcmv[1] = W20msReadADC(TP1);
      adcmv[1] -= adcmv[3];
      R_PORT = 1<<PIN_RH2;		//RH2 to VCC
      R_DDR = (1<<PIN_RH2) | (1<<PIN_RH3);	//RH3 to -
      adcmv[2] = W20msReadADC(TP2);
      adcmv[2] -= adcmv[3];
      lcd_MEM_string(RHRH_str);	// "RHRH"
   }
				//############################################
   if (tt == 4) { // Text release probes
      lcd_MEM_string(RELPROBE);	// "Release Probes"
      if (AllProbesShorted() != 0) ww = MAX_REP-2;
   }
				//############################################
   if (tt == 5) { // can we switch the ADC pins to GND across R_H resistor?
      R_PORT = 0;
      R_DDR = 1<<PIN_RH1;		//Pin 1 over R_H to GND
      adcmv[0] = W20msReadADC(TP1);

      R_DDR = 1<<PIN_RH2;		//Pin 2 over R_H to GND
      adcmv[1] = W20msReadADC(TP2);

      R_DDR = 1<<PIN_RH3;		//Pin 3 over R_H to GND
      adcmv[2] = W20msReadADC(TP3);
      lcd_MEM_string(RH1L_str);	// "RH_Lo="
   }
				//############################################
   if (tt == 6) { // can we switch the ADC pins to VCC across the R_H resistor?
      R_DDR = 1<<PIN_RH1;		//Pin 1 over R_H to VCC
      R_PORT = 1<<PIN_RH1;
      adcmv[0] = W20msReadADC(TP1) - ADCconfig.U_AVCC;
      R_DDR = 1<<PIN_RH2;		//Pin 2 over R_H to VCC
      R_PORT = 1<<PIN_RH2;
      adcmv[1] = W20msReadADC(TP2) - ADCconfig.U_AVCC;
      R_DDR = 1<<PIN_RH3;		//Pin 3 over R_H to VCC
      R_PORT = 1<<PIN_RH3;
      adcmv[2] = W20msReadADC(TP3) - ADCconfig.U_AVCC;
      lcd_MEM_string(RH1H_str);	// "RH_Hi="
   }
   if (tt == 7) { // is the voltage of all R_H / R_L dividers correct?
      u680 = ((long)ADCconfig.U_AVCC * (PIN_RM + R_L_VAL) / (PIN_RM + R_L_VAL + (unsigned long)R_H_VAL*100));
      R_PORT = 1<<PIN_RH1;		//RH1 to VCC
      R_DDR = (1<<PIN_RH1) | (1<<PIN_RL1);	//RH1 to +, RL1 to -
      adcmv[0] = W20msReadADC(TP1);
      adcmv[0] -= u680;
      R_PORT = 1<<PIN_RH2;		//RH2 to VCC
      R_DDR = (1<<PIN_RH2) | (1<<PIN_RL2);	//RH2 to +, RL2 to -
      adcmv[1] = W20msReadADC(TP2);
      adcmv[1] -= u680;
      R_PORT = 1<<PIN_RH3;		//RH3 to VCC
      R_DDR = (1<<PIN_RH3) | (1<<PIN_RL3);	//RH3 to +, RL3 to -
      adcmv[2] = W20msReadADC(TP3);
      adcmv[2] -= u680;
      lcd_MEM_string(RHRL_str);	// "RH/RL"
   }
				//############################################
   if (tt > 1) {	// output 3 voltages 
      lcd_line2();			//Cursor to column 1, row 2
      i2lcd(adcmv[0]);		// lcd_string(itoa(adcmv[0], outval, 10));	//output voltage 1
      lcd_space();
      i2lcd(adcmv[1]);		// lcd_string(itoa(adcmv[1], outval, 10));	//output voltage 2
      lcd_space();
      i2lcd(adcmv[2]);		// lcd_string(itoa(adcmv[2], outval, 10));	//output voltage 3
   }
   ADC_DDR =  TXD_MSK;		// all-Pins to Input
   ADC_PORT = TXD_VAL;		// all ADC-Ports to GND
   R_DDR = 0;			// all R-Ports to Input
   R_PORT = 0;
   taste = wait_for_key_ms(1000);	// wait up to 1 second or key is pressed
   if ((tt != 4) && (taste > 10)) {
      // don't finish repetition  for T4 with pressed key
      break; // if key is pressed, don't repeat
   }
} //end for ww
wait_for_key_ms(1000);	// wait up to 1 second or key is pressed
} //end for tt
  #if PROCESSOR_TYP == 1280
lcd_clear();
lcd_testpin(TP1);
lcd_data('L');
lcd_equal();			// lcd_data('=');
ADC_PORT = TXD_VAL;
ADC_DDR = (1<<TP1) | TXD_MSK;
R_PORT = (1<<PIN_RL1);
R_DDR = (1<<PIN_RL1);
adcmv[0] = W5msReadADC(TP1);
ADCSRB = (1<<MUX5);		// switch to upper 8 MUX inputs
adcmv[1] = ReadADC(PIN_RL1);
ADCSRB = 0;			// switch back to lower 8 MUX inputs
ResistorVal[0] = (adcmv[0] * (unsigned long)R_L_VAL) / (adcmv[1] - adcmv[0]);
DisplayValue(ResistorVal[0],-1,LCD_CHAR_OMEGA,3);
lcd_space();
lcd_data('H');
lcd_equal();			// lcd_data('=');
ResistorVal[1] = (vcc_diff(adcmv[1]) * (unsigned long)R_L_VAL) / (adcmv[1] - adcmv[0]);
DisplayValue(ResistorVal[1],-1,LCD_CHAR_OMEGA,3);
lcd_line2();
lcd_testpin(TP1);
lcd_space();
lcd_data('H');
lcd_equal();			// lcd_data('=');
ADC_PORT = (1<<TP1) | TXD_VAL;
R_PORT = 0;
adcmv[0] = W5msReadADC(TP1);
ADCSRB = (1<<MUX5);		// switch to upper 8 MUX inputs
adcmv[1] = ReadADC(PIN_RL1);
ADCSRB = 0;			// switch back to lower 8 MUX inputs
ResistorVal[1] = (vcc_diff(adcmv[0]) * (unsigned long)R_L_VAL) / (adcmv[0] - adcmv[1]);
DisplayValue(ResistorVal[1],-1,LCD_CHAR_OMEGA,3);
lcd_space();
lcd_data('L');
lcd_equal();			// lcd_data('=');
ResistorVal[0] = (adcmv[1] * (unsigned long)R_L_VAL) / (adcmv[0] - adcmv[1]);
DisplayValue(ResistorVal[0],-1,LCD_CHAR_OMEGA,3);
wait_about1s();			// only for mega1280
last_line_used = 2;
wait_for_key_5s_line2();		// wait up to 5 seconds and clear line 2
// 
lcd_clear();
lcd_testpin(TP2);
lcd_data('L');
lcd_equal();			// lcd_data('=');
ADC_PORT = TXD_VAL;
ADC_DDR = (1<<TP2) | TXD_MSK;
R_PORT = (1<<PIN_RL2);
R_DDR = (1<<PIN_RL2);
adcmv[0] = W5msReadADC(TP2);
ADCSRB = (1<<MUX5);		// switch to upper 8 MUX inputs
adcmv[1] = ReadADC(PIN_RL2);
ADCSRB = 0;			// switch back to lower 8 MUX inputs
ResistorVal[0] = (adcmv[0] * (unsigned long)R_L_VAL) / (adcmv[1] - adcmv[0]);
DisplayValue(ResistorVal[0],-1,LCD_CHAR_OMEGA,3);
lcd_space();
lcd_data('H');
lcd_equal();			// lcd_data('=');
ResistorVal[1] = (vcc_diff(adcmv[1]) * (unsigned long)R_L_VAL) / (adcmv[1] - adcmv[0]);
DisplayValue(ResistorVal[1],-1,LCD_CHAR_OMEGA,3);
lcd_line2();
lcd_testpin(TP2);
lcd_data('H');
lcd_equal();			// lcd_data('=');
ADC_PORT = (1<<TP2) | TXD_VAL;
R_PORT = 0;
adcmv[0] = W5msReadADC(TP2);
ADCSRB = (1<<MUX5);		// switch to upper 8 MUX inputs
adcmv[1] = ReadADC(PIN_RL2);
ADCSRB = 0;			// switch back to lower 8 MUX inputs
ResistorVal[1] = (vcc_diff(adcmv[0]) * (unsigned long)R_L_VAL) / (adcmv[0] - adcmv[1]);
DisplayValue(ResistorVal[1],-1,LCD_CHAR_OMEGA,3);
lcd_space();
lcd_data('L');
lcd_equal();			// lcd_data('=');
ResistorVal[0] = (adcmv[1] * (unsigned long)R_L_VAL) / (adcmv[0] - adcmv[1]);
DisplayValue(ResistorVal[0],-1,LCD_CHAR_OMEGA,3);
wait_about1s();			// only for mega1280
last_line_used = 2;
wait_for_key_5s_line2();		// wait up to 5 seconds and clear line 2
//
lcd_clear();
lcd_testpin(TP3);
lcd_data('L');
lcd_equal();			// lcd_data('=');
ADC_DDR = (1<<TP3) | TXD_MSK;
R_PORT = (1<<PIN_RL3);
R_DDR = (1<<PIN_RL3);
adcmv[0] = W5msReadADC(TP3);
ADCSRB = (1<<MUX5);		// switch to upper 8 MUX inputs
adcmv[1] = ReadADC(PIN_RL3);
ADCSRB = 0;
ResistorVal[0] = (adcmv[0] * (unsigned long)R_L_VAL) / (adcmv[1] - adcmv[0]);
DisplayValue(ResistorVal[0],-1,LCD_CHAR_OMEGA,3);
lcd_space();
lcd_data('H');
lcd_equal();			// lcd_data('=');
ResistorVal[1] = (vcc_diff(adcmv[1]) * (unsigned long)R_L_VAL) / (adcmv[1] - adcmv[0]);
DisplayValue(ResistorVal[1],-1,LCD_CHAR_OMEGA,3);
lcd_line2();
lcd_testpin(TP3);
lcd_data('H');
lcd_equal();			// lcd_data('=');
ADC_PORT = (1<<TP3) | TXD_VAL;
R_PORT = 0;
adcmv[0] = W5msReadADC(TP3);
ADCSRB = (1<<MUX5);		// switch to upper 8 MUX inputs
adcmv[1] = ReadADC(PIN_RL3);
ADCSRB = 0;			// switch back to lower 8 MUX inputs
ResistorVal[1] = (vcc_diff(adcmv[0]) * (unsigned long)R_L_VAL) / (adcmv[0] - adcmv[1]);
DisplayValue(ResistorVal[1],-1,LCD_CHAR_OMEGA,3);
lcd_space();
lcd_data('L');
lcd_equal();			// lcd_data('=');
ResistorVal[0] = (adcmv[1] * (unsigned long)R_L_VAL) / (adcmv[0] - adcmv[1]);
DisplayValue(ResistorVal[0],-1,LCD_CHAR_OMEGA,3);
wait_about1s();			// only for mega1280
last_line_used = 2;
wait_for_key_5s_line2();		// wait up to 5 seconds and clear line 2
  #endif	/* PROCESSOR_TYP == 1280 */
}	/* end if((test_mode & 0x0f) == 1) */
 #endif		/* end EXTENDED_TESTS */

for (ww=0;ww<120;ww++) {
  // wait up to 1 minute for releasing the probes
  if (AllProbesShorted() == 0) break;
  lcd_clear_line2();		// clear total line2
  lcd_MEM_string(RELPROBE);	// "Release Probes"
  lcd_refresh();		// write the pixels to display, ST7920 only
  wait_about500ms();
}



lcd_clear();
lcd_MEM_string(RIHI_str);	// "RiHi="
DisplayValue16(RRpinPL,-1,LCD_CHAR_OMEGA,3);
lcd_line2();
lcd_MEM_string(RILO_str);	// "RiLo="
DisplayValue16(RRpinMI,-1,LCD_CHAR_OMEGA,3);
last_line_used = 2;
wait_for_key_5s_line2();		// wait up to 5 seconds and clear line 2

//measure Zero offset for Capacity measurement
PartFound = PART_NONE;
lcd_clear();
lcd_MEM_string(C0_str);			//output "C0 "
ReadCapacity(TP3, TP1);
adcmv[5] = (unsigned int) cap.cval_uncorrected.dw;	//save capacity value of empty Pin 1:3
ReadCapacity(TP3, TP2);
adcmv[6] = (unsigned int) cap.cval_uncorrected.dw;	//save capacity value of empty Pin 2:3
ReadCapacity(TP2, TP1);
adcmv[2] = (unsigned int) cap.cval_uncorrected.dw;	//save capacity value of empty Pin 1:2
ReadCapacity(TP1, TP3);
adcmv[1] = (unsigned int) cap.cval_uncorrected.dw;	//save capacity value of empty Pin 3:1
ReadCapacity(TP2, TP3);
adcmv[4] = (unsigned int) cap.cval_uncorrected.dw;	//save capacity value of empty Pin 3:2
ReadCapacity(TP1, TP2);
adcmv[0] = (unsigned int) cap.cval_uncorrected.dw;	//save capacity value of empty Pin 2:1
 #ifdef WITH_MENU
if (((test_mode & 0x0f) == 1) || (UnCalibrated == 2))
 #else
if (UnCalibrated == 2)
 #endif
{
  adcmv[3] = adcmv[0] + 2;		// mark as uncalibrated until Cap > 100nF has success
} else {
  adcmv[3] = adcmv[0];			// mark as calibrated, short calibration is finished
  UnCalibrated = 0;			// clear the UnCalibrated Flag
  lcd_cursor_off();			// switch cursor off
}
u2lcd_space(adcmv[5]);	//DisplayValue(adcmv[5],0,' ',3);		//output cap0 1:3
u2lcd_space(adcmv[6]);	//DisplayValue(adcmv[6],0,' ',3);		//output cap0 2:3
DisplayValue(adcmv[2],-12,'F',3);		//output cap0 1:2
 #ifdef AUTO_CAL
for (ww=0;ww<7;ww++) {			//checking loop
if ((adcmv[ww] > 190) || (adcmv[ww] < 10)) goto no_c0save;
}
for (ww=0;ww<7;ww++) {
  // write all zero offsets to the EEprom 
  (void) eeprom_write_byte((uint8_t *)(&c_zero_tab[ww]),adcmv[ww]+(COMP_SLEW1 / (CC0 + CABLE_CAP + COMP_SLEW2)));
}
lcd_line2();
lcd_MEM_string(OK_str);		// output "OK"
no_c0save:
 #endif
last_line_used = 2;
wait_for_key_5s_line2();		// wait up to 5 seconds and clear line 2

#ifdef SamplingADC
  sampling_cap_calibrate();		// measure zero capacity for samplingADC
#endif

 #ifdef AUTO_CAL
  #ifdef WITH_MENU
if (((test_mode & 0x0f) == 1) || (UnCalibrated == 2))
  #endif
// without menu function the capacitor is requested every time,
// because there is no way to request recaltbration again
// With menu function the capacitor is only requested for first time 
// calibration (UnCalibrated = 2) or for the full selftest call (test_mode = 1) 
// of the menu function, not with the automatically call (test_mode = 1).
{
// for full test or first time calibration, use external capacitor
// Message C > 100nF at TP1 and TP3
cap_found = 0;
#if (TPCAP <= 0)
lcd_clear();
lcd_testpin(TP1);
lcd_MEM_string(CapZeich);	// "-||-"
lcd_testpin(TP3);
lcd_MEM2_string(MinCap_str);	// " >100nF!"
#endif
for (ww=0;ww<64;ww++) {
  init_parts();
  #if (TPCAP >= 0)
  CalibrationCap();	// measure with internal calibration capacitor
  #else
  PartFound = PART_NONE;
  ReadCapacity(TP3, TP1);	// look for capacitor > 100nF
  #endif
  while (cap.cpre < -9) {
   cap.cpre++;
   cap.cval /= 10;
  }
  if ((cap.cpre == -9) && (cap.cval > 95) && (cap.cval < 22000) &&
    (load_diff > -150) && (load_diff < 150)) {
   cap_found++;
  } else {
   cap_found = 0;		// wait for stable connection
  }
  if (cap_found > 4) {
     // value of capacitor is correct
     (void) eeprom_write_word((uint16_t *)(&ref_offset), load_diff);	// hold zero offset + slew rate dependend offset
     lcd_clear();
     lcd_MEM2_string(REF_C_str);	// "REF_C="
     i2lcd(load_diff);		// lcd_string(itoa(load_diff, outval, 10));	//output REF_C_KORR
     RefVoltage();			// new ref_mv_offs and RHmultip
  #if 0
//#######################################
   // Test for switching level of the digital input of port TP3
   for (tt=0;tt<8;tt++) {
     ADC_PORT =  TXD_VAL;	//ADC-Port 1 to GND
     ADC_DDR = 1<<TP1 | TXD_MSK;	//ADC-Pin  1 to output 0V
     R_PORT = 1<<PIN_RH3;		//Pin 3 over R_H to VCC
     R_DDR = 1<<PIN_RH3;		//Pin 3 over R_H to VCC
     while (1) {
        wdt_reset();
        if ((ADC_PIN&(1<<TP3)) == (1<<TP3)) break;
     }
     R_DDR = 0;		//Pin 3 without current
     R_PORT = 0;
     adcmv[0] = ReadADC(TP3);
     lcd_line3();
     Display_mV(adcmv[0],4);
     R_DDR = 1<<PIN_RH3;		//Pin 3 over R_H to GND
     while (1) {
        wdt_reset();
        if ((ADC_PIN&(1<<TP3)) != (1<<TP3)) break;
     }
     R_DDR = 0;		//Pin 3 without current
     lcd_line4();
     adcmv[0] = ReadADC(TP3);
     Display_mV(adcmv[0],4);
     last_line_used = 2;
     wait_for_key_5s_line2();		// wait up to 5 seconds and clear line 2
   }
//#######################################
  #endif
  #ifdef AUTOSCALE_ADC
   #if (TPCAP >= 0)
    #define CAP_ADC TPCAP	/* Cap >100nF is integrated at TPCAP */
   TCAP_PORT &= ~(1<<TCAP_RH);	// 470k resistor to GND
   TCAP_DDR |= (1<<TCAP_RH);	// enable output
   #else
    #define CAP_ADC TP3		/* Cap >100nF at TP3 */
   ADC_PORT =  TXD_VAL;	//ADC-Port 1 to GND
   ADC_DDR = 1<<TP1 | TXD_MSK;	//ADC-Pin  1 to output 0V
   R_DDR = 1<<PIN_RH3;		//Pin 3 over R_H to GND
   #endif
   do {
      adcmv[0] = ReadADC(CAP_ADC);
   } while (adcmv[0] > 980);
   #if (TPCAP >= 0)
   TCAP_DDR &= ~(1<<TCAP_RH);	// 470k resistor port to input mode
   #else
   R_DDR = 0;		//all Pins to input 
   #endif
   ADCconfig.U_Bandgap = 0;	// do not use internal Ref
   adcmv[0] = ReadADC(CAP_ADC);  // get cap voltage with VCC reference
   ADCconfig.U_Bandgap = adc_internal_reference;
   adcmv[1] = ReadADC(CAP_ADC);	// get cap voltage with internal reference
   adcmv[1] += adcmv[1];		// double the value
   ADCconfig.U_Bandgap = 0;	// do not use internal Ref
   adcmv[2] = ReadADC(CAP_ADC);  // get cap voltage with VCC reference
   ADCconfig.U_Bandgap = adc_internal_reference;
   udiff = (int8_t)(((signed long)(adcmv[0] + adcmv[2] - adcmv[1])) * adc_internal_reference / adcmv[1])+REF_R_KORR;
   lcd_line2();
   lcd_MEM2_string(REF_R_str);	// "REF_R="
   udiff2 = udiff + (int8_t)eeprom_read_byte((uint8_t *)(&RefDiff));
   (void) eeprom_write_byte((uint8_t *)(&RefDiff), (uint8_t)udiff2);	// hold offset for true reference Voltage
   i2lcd(udiff2);		// output correction voltage
   RefVoltage();			// set new ADCconfig.U_Bandgap
  #endif	/* end AUTOSCALE_ADC */
   last_line_used = 2;
   wait_for_key_5s_line2();		// wait up to 5 seconds and clear line 2
   UnCalibrated = 0;		// clear the UnCalibrated Flag
   lcd_cursor_off();		// switch cursor off
   cap_found = eeprom_read_byte((uint8_t *)&c_zero_tab[0]);	// read first capacity zero offset
   eeprom_write_byte((uint8_t *)&c_zero_tab[3], cap_found);	// mark as calibrated permanent
   break;			// leave the ww for loop
  }  /* end if (cap_found > 4) */
  lcd_line2();
  DisplayValue(cap.cval,cap.cpre,'F',4);
  lcd_clear_line();
  lcd_refresh();		// write the pixels to display, ST7920 only
  wait_about200ms();			// wait additional time
} // end for ww
}	/* end if((test_mode & 0x0f) == 1) */
 #endif  /* end AUTO_CAL */

ADCconfig.Samples = ANZ_MESS;	// set to configured number of ADC samples

#if defined(SamplingADC) && !defined(AUTO_LC_CAP)
  sampling_lc_calibrate(0);	// Cap for L-meas
#endif


 #ifdef FREQUENCY_50HZ
//#define TEST_SLEEP_MODE	/* only select for checking the sleep delay */
 lcd_clear();
 lcd_MEM_string(T50HZ);	//" 50Hz"
 lcd_refresh();		// write the pixels to display, ST7920 only
 ADC_PORT = TXD_VAL;
 ADC_DDR = 1<<TP1 | TXD_MSK;	// Pin 1 to GND
 R_DDR = (1<<PIN_RL3) | (1<<PIN_RL2);
 for(ww=0;ww<30;ww++) {	// repeat the signal up to 30 times (1 minute)
   for (ff50=0;ff50<100;ff50++) {	// for 2 s generate 50 Hz
      R_PORT = (1<<PIN_RL2);	// Pin 2 over R_L to VCC, Pin 3 over R_L to GND
  #ifdef TEST_SLEEP_MODE
      sleep_5ms(2); 		// test of timing of sleep mode call 
  #else
      wait10ms();		// normal delay
  #endif
      R_PORT = (1<<PIN_RL3);	// Pin 3 over R_L to VCC, Pin 2 over R_L to GND
  #ifdef TEST_SLEEP_MODE
      sleep_5ms(2); 		// test of timing of sleep mode call 
  #else
      wait10ms();		// normal delay
  #endif
      wdt_reset();
   } /* end for ff50 */
   if (!(RST_PIN_REG & (1<<RST_PIN))) {
      // if key is pressed, don't repeat
      break;
   }
 } /* end for ww */
 #endif		/* end FREQUENCY_50HZ */
lcd_clear();
lcd_MEM_string(VERSION_str);	//"Version ..."
lcd_line2();
lcd_MEM_string(ATE);		//"Selftest End"
PartFound = PART_NONE;
     last_line_used = 2;
wait_for_key_5s_line2();		// wait up to 5 seconds and clear line 2
} /* end AutoCheck */ 

 
/*
 *  check for a short circuit between two probes
 *  from Markus R.
 *
 *  requires:
 *  - ID of first probe (0-2)
 *  - ID of second probe (0-2)
 *
 *  returns:
 *  - 0 if not shorted
 *  - 1 if shorted
 */

uint8_t ShortedProbes(uint8_t Probe1, uint8_t Probe2)
{
  uint8_t           Flag1 = 0;      /* return value */
  unsigned int      U1;            /* voltage at probe #1 in mV */
  unsigned int      U2;            /* voltage at probe #2 in mV */
  unsigned int      URH;	   /* half of reference voltage */
  const uint8_t *addr;
  uint8_t pp;
  /*
   *  Set up a voltage divider between the two probes:
   *  - Probe1: Rl pull-up
   *  - Probe2: Rl pull-down
   */

  ADC_DDR =  TXD_MSK;		// all-Pins to Input
  ADC_PORT = TXD_VAL;		// all ADC-Ports to GND
  addr = &PinRLRHADCtab[Probe1];
  pp = pgm_read_byte(addr);
  R_PORT = pp;
  addr += (int8_t)(Probe2-Probe1);
  R_DDR =  pp | pgm_read_byte(addr);	// pgm_read_byte(PinRHtab[Probe1]) | pgm_read_byte(PinRLtab[Probe2]);

  /* read voltages */
  U1 = ReadADC(Probe1);
  U2 = ReadADC(Probe2);

  /*
   *  We expect both probe voltages to be about the same and
   *  to be half of Vcc (allowed difference +/- 20mV).
   */
 #ifndef MAX_UH_DIFF
  #define MAX_UH_DIFF 30
 #endif

  URH = ADCconfig.U_AVCC / 2;
  URH -= ((long)U_VCC * (long)(PIN_RP-PIN_RM)) / (4*(unsigned long)(R_L_VAL+PIN_RM));			// differenz of Pin resistance high (22) and low (20)
  if (((U1 + MAX_UH_DIFF) > URH ) && (U1 < (URH + MAX_UH_DIFF)))
  {
    if (((U2 + MAX_UH_DIFF) > URH) && (U2 < (URH + MAX_UH_DIFF)))
    {
      Flag1 = 1;
    }
  }

  /* reset port */
  R_DDR = 0;

  return Flag1;
}
/*
 *  check for a short circuit between all probes
 *  from Markus R.
 *
 *  returns:
 *  - 0 if no probes are short-circuited
 *  - number of probe pairs short-circuited (3 = all)
 */

uint8_t AllProbesShorted(void)
{
  uint8_t           Flag2;      /* return value */

  /* check all possible combinations */
  Flag2 = ShortedProbes(TP1, TP2);
  Flag2 += ShortedProbes(TP1, TP3);
  Flag2 += ShortedProbes(TP2, TP3);

  return Flag2;
}
#endif /* end #ifdef WITH_SELFTEST */
/* End AutoCheck.c */
#endif


#ifdef AUTO_CAL
 /* Begin mark_as_uncalibrated.c */
void mark_as_uncalibrated( void) {
 uint8_t ii;
 if (!UnCalibrated) {
    // equipment has changed, zero capacity value is too high or zero resistance is too high
    // but the device is marked as Calibrated.
    ii = eeprom_read_byte((uint8_t *)&c_zero_tab[0]);	// read first zero offset
    ii++;
    // make the unused c_zero_tab[3] different to c_zero_tab[0] to mark uncalibrated 
    eeprom_write_byte((uint8_t *)&c_zero_tab[3], ii);	// mark uncalibrated permanent
    UnCalibrated = 1;		// set back to uncalibrated
 }
}
/* End mark_as_uncalibrated.c */
#endif


#if FLASHEND > 0x1fff
 /* Begin GetIr.c */
/* Get residual current in reverse direction of a diode */ 
//=================================================================
void GetIr(uint8_t hipin, uint8_t lopin) {
  unsigned int u_res;	// reverse voltage at 470k
  unsigned int u_res_old;
  unsigned int ir_nano;
  uint8_t HiADC;

#if (((PIN_RL1 + 1) != PIN_RH1) || ((PIN_RL2 + 1) != PIN_RH2) || ((PIN_RL3 + 1) != PIN_RH3))
  HiADC = pgm_read_byte(&PinRLRHADCtab[hipin-TP_MIN]+6);	// Table of ADC pins including | TXD_VAL
  ADC_PORT = HiADC;		 	// switch ADC port to high level
  ADC_DDR = HiADC | TXD_MSK;		// switch High Pin direct to VCC
  R_PORT = 0;				// switch R-Port to GND
  LoPinR_L = pgm_read_byte(&PinRHRLADCtab[lopin-TP_MIN]);  //R_L mask for LowPin R_L load
#else
  uint8_t LoPinR_L;
  HiADC = pgm_read_byte(&PinRLRHADCtab[hipin-TP_MIN]+3);	// Table of ADC pins including | TXD_VAL
  ADC_PORT = HiADC;		 	// switch ADC port to high level
  ADC_DDR = HiADC | TXD_MSK;		// switch High Pin direct to VCC
  R_PORT = 0;				// switch R-Port to GND
  LoPinR_L = pgm_read_byte(&PinRLRHADCtab[lopin-TP_MIN]);  //R_L mask for LowPin R_L load
  // R_H Pin must always be one pin number higher
#endif
  R_DDR = LoPinR_L;		// switch R_L port for LowPin to output (GND)
  u_res = U_VCC;
  // first load the parallel capacity with 680 Ohm resistor, then measure current with 470k
  do {
     u_res_old = u_res;
     u_res = W20msReadADC(lopin);		// read voltage
#if (((PIN_RL1 + 1) != PIN_RH1) || ((PIN_RL2 + 1) != PIN_RH2) || ((PIN_RL3 + 1) != PIN_RH3))
     R_DDR = pgm_read_byte(&PinRHRLADCtab[lopin-TP_MIN+3]);  //R_H mask for LowPin R_H load
#else
     R_DDR = LoPinR_L + LoPinR_L;		// switch R_H port for LowPin to output (GND)
#endif
  } while (u_res < u_res_old);
  if (u_res == 0) return;		// no Output, if no current in reverse direction
#if (LCD_LINES > 3)
 #define IR_DIGITS 3
  lcd_line4();				// use Line 4 for Ir output
  lcd_MEM_string(Ir_str);		// output text "Ir="
#else
 #define IR_DIGITS 2
  lcd_MEM_string(Ir_str);		// output text "  Ir="
#endif
#ifdef WITH_IRMICRO
  unsigned int ir_micro;
  if (u_res < 2500) {
#endif
     /* R_H_VAL has units of 10 Ohm, u_res has units of mV, ir_nano has units of nA */
     ir_nano = (unsigned long)(u_res * 100000UL) / R_H_VAL;
     DisplayValue16(ir_nano,-9,'A',2);	// output two digits of current with nA units
#ifdef WITH_IRMICRO
  } else {
     R_DDR = LoPinR_L;			// switch R_L port for LowPin to output (GND)
     u_res = W5msReadADC(lopin);	// read voltage
     ir_nano = 0xffff;			// set to max
     /* RR680MI has units of 0.1 Ohm, u_res has units of mV, ir_micro has units of uA */
     ir_micro = (unsigned long)(u_res * 10000UL) / RR680MI;
     DisplayValue16(ir_micro,-6,'A',IR_DIGITS);	// output 2 or 3 digits of current in uA units
  }
#endif
  ADC_DDR = TXD_MSK;			// switch off
  ADC_PORT = TXD_VAL;			// switch off
  R_DDR = 0;				// switch off current

  return ;
}
/* End GetIr.c */
#endif
  
//}


void loop() {
    
}
