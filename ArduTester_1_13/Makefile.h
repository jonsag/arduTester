/********************************************************************************
* Makefile variables settings from Makefile Arduino/Genuino UNO  or Mega 2560   *
*********************************************************************************/


//#define PARTNO m2560 // must for Mega2560

//#define ARDUINO_MEGA // resolves pin assignement in config.h for pinMode() and digitalWrite()

#define ARDUINO_UNO // to run on the UNO with pighixxx hardware definitions

// if no LCD specified, its 4 bits parallel
// with LCD 7920 serial
//#define FOUR_LINE_LCD 0
//#define WITH_LCD_ST7565 7920
//#define LCD_INTERFACE_MODE 5

// if serial monitor, disables LCD
//#define WITH_HARDWARE_SERIAL
//#define BAUDRATE 115200
//#define WITH_UART 0

#define WITH_MENU 0 // use only with NO_FREQ_COUNTER
#define NO_FREQ_COUNTER // must with UNO Board

#define UI_LANGUAGE LANG_ENGLISH
#define WITH_SELFTEST
#define AUTO_CAL
#define SHORT_UNCAL_MSG

#define WITH_SamplingADC 1
#define SamplingADC
#define FET_Idss
#define WITH_AUTO_REF
#define REF_C_KORR 12
#define REF_L_KORR 40
#define C_H_KORR 0

//#define WITH_UART 0

#define RMETER_WITH_L 0 
#define CAP_EMPTY_LEVEL 4
#define AUTOSCALE_ADC
#define REF_R_KORR 3
#define ESR_ZERO 20
#define OP_MHZ 16
#define MHZ_CPU OP_MHZ 
#define PULLUP_DISABLE
#define ANZ_MESS 25
#define POWER_OFF 0 
#define BAT_OUT 150
#define BAT_POOR 6400
#define INHIBIT_SLEEP_MODE 1

//#define VEXT

// if other RH , RL resistors
//#define R_H_VAL 10000 // 100kOhm instead of 470k
//#define R_L_VAL 3300  // 330 Ohm instead of 680

