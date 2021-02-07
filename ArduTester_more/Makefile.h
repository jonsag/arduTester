/***************************************************************
* Makefile Options setting Arduino/Genuino UNO  or Mega 2560   *
***************************************************************/

// current file is set for MEGA with OLED 1306 I2C


/* Board selection  UNO or MEGA */
#define PARTNO m2560 // uncomment in case of MEGA 2560 board
//#define ARDUINO_UNO // uncomment in case of UNO board
//#define NO_FREQ_COUNTER // uncomment when UNO Board , FREQ counter function only with MEGA 2560 Board


/* if no Display at all, use Serial Monitor Output
   uncomment these lines */
//#define WITH_HARDWARE_SERIAL
//#define BAUDRATE 115200
//#define WITH_UART 0


/* if no Serial Monitor and no Graphic Display below uncommented
, its 4 bits parallel mode 1602 LCD by default */


/* if the 1602 LCD has a I2C interface 
uncomment these 2 line */
//#define WITH_1602_I2C
//#define LCD_I2C_ADDR 0x27


/* with OLED 1306  I2C interface uncomment this lines */
#define FOUR_LINE_LCD 0
#define WITH_LCD_ST7565 1306
#define LCD_INTERFACE_MODE 2
#define LCD_ST_TYPE 1306
#define LCD_I2C_ADDR 0x3c


/* with OLED 1306  SPI interface uncomment this lines */
//#define FOUR_LINE_LCD 0
//#define WITH_LCD_ST7565 1306
//#define LCD_ST_TYPE 1306
//#define LCD_ST7565_RESISTOR_RATIO 4


/* with LCD 7920 serial, uncomment next 3 lines */
//#define FOUR_LINE_LCD 0
//#define WITH_LCD_ST7565 7920
//#define LCD_INTERFACE_MODE 5


/* with LCD ST 7735, uncomment next lines */ 
//#define FOUR_LINE_LCD 0
//#define WITH_LCD_ST7565 1
//#define LCD_ST_TYPE 7735 
//#define LCD_ST7565_RESISTOR_RATIO 4
//#define FONT_8X12thin
//#define ICON_TYPE 2
//#define BIG_TP
//#define LCD_INTERFACE_MODE MODE_SPI
//#define VOLUME_VALUE 25
//#define DLCD_ST7565_V_FLIP 1
//#define DLCD_ST7565_H_FLIP 1
//#define DLCD_ST7565_H_OFFSET 4


/* with LCD 7108 display uncomment this lines */
//#define FOUR_LINE_LCD 0
//#define WITH_LCD_ST7565 7108
//#define ST_CS_LOW
//#define LCD_INTERFACE_MODE MODE_7108_SERIAL
//#define LCD_INTERFACE_MODE MODE_SPI


/* with LCD 1327  I2C */
//#define FOUR_LINE_LCD 0
//#define WITH_LCD_ST7565 1327
//#define LCD_INTERFACE_MODE 2
//#define LCD_I2C_ADDR 0x3d
//#define LCD_ST_TYPE 1327


#define WITH_MENU 0 
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
