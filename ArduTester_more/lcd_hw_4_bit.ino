#include "Wire.h"
#include "SPI.h"
//#include "LiquidCrystal_I2C.h"

#define FAST_SPI_OUTPUT

#if (LCD_INTERFACE_MODE == MODE_SPI) || (LCD_INTERFACE_MODE == MODE_3LINE)

// serial output for ST7565 controller, 4-Bit SPI
//_lcd_hw_write:

void _lcd_hw_write(unsigned char preg_1, unsigned char preg_2){
 #ifdef LCD_SPI_OPEN_COL
//	set_en_low
//	set_en_output		// en/CLK to GND
digitalWrite(HW_LCD_EN_PIN,LOW);
pinMode(HW_LCD_EN_PIN,OUTPUT);
  #ifdef PULLUP_DISABLE
//	AOUT	MCUCR, r1		; MCUCR = 0;	//enable pull up resistors
//  MCUCR=0x00;
// asm volatile (
//    "AOUT MCUCR, r1\n"
//    );
  #endif
//	set_ce_low
//	set_ce_output		// enable chip
digitalWrite(HW_LCD_CE_PIN,LOW);
pinMode(HW_LCD_CE_PIN,OUTPUT);

//; Set RS (0=Cmd, 1=Char)	
  #if (LCD_INTERFACE_MODE == MODE_3LINE)
//	sbrs    preg_1, 0
//	rjmp	clr_rs
	if ((preg_1&1)==0) { 
	  digitalWrite(HW_LCD_B0_PIN,LOW);
    pinMode(HW_LCD_B0_PIN,OUTPUT);}
//	set_b0_input
//	set_b0_high		// enable B0 pullup
  else pinMode(HW_LCD_B0_PIN,INPUT_PULLUP);
//  {pinMode(HW_LCD_B0_PIN,INPUT);digitalWrite(HW_LCD_B0_PIN,HIGH);}
//	rjmp	set_sce
//clr_rs:
//	set_b0_low 
//	set_b0_output
//set_sce:

//	set_rs_low
//	set_rs_output		// SCE to GND;
//	rcall	wait1us
digitalWrite(HW_LCD_RS_PIN,LOW);
pinMode(HW_LCD_RS_PIN,OUTPUT);
 void wait1us();
 
//	set_en_input
//	set_en_high		// enable en pullup
pinMode(HW_LCD_EN_PIN,INPUT_PULLUP);
//pinMode(HW_LCD_EN_PIN,INPUT);
//digitalWrite(HW_LCD_EN_PIN,HIGH);

	
//	rcall	wait1us
 void wait1us();
  #else  // (LCD_INTERFACE_MODE == MODE_3LINE)
//	sbrs    preg_1, 0
//	rjmp	clr_rs
//	set_rs_input		// set B0 to input
//	set_rs_high		// enable B0 pullup
//        rjmp	fini_rs
//clr_rs:
//	set_rs_low
//	set_rs_output		// set B0 for RS to GND
//fini_rs:
//	rcall	wait1us
	if ((preg_1&1)==0) {
	  digitalWrite(HW_LCD_RS_PIN,LOW);
    pinMode(HW_LCD_RS_PIN,OUTPUT);}
  else pinMode(HW_LCD_RS_PIN,INPUT_PULLUP);
 //{pinMode(HW_LCD_RS_PIN,INPUT);digitalWrite(HW_LCD_RS_PIN,HIGH);}
  void wait1us();
  #endif //(LCD_INTERFACE_MODE == MODE_3LINE)

//; Send bit-7
//      ROL	preg_2		// shift B7 to carry
//      rcall shift_out
        shift_out(preg_2 & 128);
//; Send bit-6
//      ROL	preg_2		// shift B6 to carry
//      shift_out()
        shift_out(preg_2 & 64);
//; Send bit-5
//        ROL	preg_2		// shift B5 to carry
//        rcall	shift_out
          shift_out(preg_2 & 32);
//; Send bit-4
//        ROL	preg_2		// shift B4 to carry
//        rcall	shift_out
          shift_out(preg_2 & 16);
//; Send bit-3
//        ROL	preg_2		// shift B3 to carry
//        rcall	shift_out
          shift_out(preg_2 & 8);
//; Send bit-2
//        ROL	preg_2		// shift B2 to carry
//        rcall	shift_out
          shift_out(preg_2 & 4);
//; Send bit-1
//        ROL	preg_2		// shift B1 to carry
//        rcall	shift_out
          shift_out(preg_2 & 2);
//; Send bit-0
//        ROL	preg_2		// shift B0 to carry
//        rcall	shift_out
          shift_out(preg_2 & 1);
//	rcall	wait1us
    void wait1us();
//	set_en_low
//	set_en_output		// set en/clk to GND
digitalWrite(HW_LCD_EN_PIN,LOW);
pinMode(HW_LCD_EN_PIN,OUTPUT);

  #if (LCD_INTERFACE_MODE == MODE_3LINE)
//	rcall	wait1us
    void wait1us();
//	set_rs_input		// SCE to  high
//	set_rs_high		// enable pullup
    pinMode(HW_LCD_RS_PIN,INPUT_PULLUP);
//pinMode(HW_LCD_RS_PIN,INPUT);
//digitalWrite(HW_LCD_RS_PIN,HIGH);
  #endif
//	set_ce_input
//        set_ce_high		// disable chip
pinMode(HW_LCD_CE_PIN,INPUT_PULLUP);
//pinMode(HW_LCD_CE_PIN,INPUT);
//digitalWrite(HW_LCD_CE_PIN,HIGH);
  #ifdef PULLUP_DISABLE
//	ldi	r25, (1<<PUD)		;
//	AOUT	MCUCR, r25		; MCUCR = (1<<PUD);	//disable pull up resistors
//    MCUCR=(1<<PUD);
// asm volatile (
//    "ldi	r25, (1<<PUD)	\n"
//    );
  #endif
//	set_en_low
//	set_en_output		// en/CLK to GND
digitalWrite(HW_LCD_EN_PIN,LOW);
pinMode(HW_LCD_EN_PIN,OUTPUT);
//	set_b0_low		// ## reset b0 to GND to prevent incorrect detection of rotary encoder movement
//	set_b0_output		// ##
digitalWrite(HW_LCD_B0_PIN,LOW);
pinMode(HW_LCD_B0_PIN,OUTPUT);
//	ret		// return _lcd_hw_write
}


// sub-function shift_out: send 1, if carry is set, send 0, if carry is reset
void shift_out(unsigned char rol_byte){
//	set_en_low
//	set_en_output		// set en/clk to GND
digitalWrite(HW_LCD_EN_PIN,LOW);
pinMode(HW_LCD_EN_PIN,OUTPUT);
//  brcc	clr_bit
//	set_b0_input		// set B0 to input
//	set_b0_high		// enable B0 pullup = high
if (rol_byte==0){
  digitalWrite(HW_LCD_B0_PIN,LOW);
  pinMode(HW_LCD_B0_PIN,OUTPUT);}
else  pinMode(HW_LCD_B0_PIN,INPUT_PULLUP);
//{pinMode(HW_LCD_B0_PIN,INPUT);digitalWrite(HW_LCD_B0_PIN,HIGH);}
//	rjmp	fini_bit
//clr_bit:
//	set_b0_low
//	set_b0_output		// set B0 for Bx to GND

//fini_bit:
//	set_en_input
//	set_en_high		// enable en/clk pullup
//	rcall	wait1us
    void wait1us();
//	ret
}	
	

	
 #else // no LCD_SPI_OPEN_COL 
  #ifdef FAST_SPI_OUTPUT
//               ; Set RS (0=Cmd, 1=Char)
//	set_ce_low
//	set_ce_output		// enable chip
    digitalWrite(HW_LCD_CE_PIN,LOW);
    pinMode(HW_LCD_CE_PIN,OUTPUT);
   #if (LCD_INTERFACE_MODE == MODE_3LINE)
//             set_en_low
               digitalWrite(HW_LCD_EN_PIN,LOW);
               //sbrc    preg_1, 0
               //set_b0_high
               if ((preg_1&1)==1) digitalWrite(HW_LCD_B0_PIN,HIGH);
               //sbrs    preg_1, 0
               //set_b0_low
               if ((preg_1&1)==0) digitalWrite(HW_LCD_B0_PIN,LOW);
               //set_b0_output		; set B0 to output
               pinMode(HW_LCD_B0_PIN,OUTPUT);
//               set_rs_low		; SCE to GND
//               set_rs_output		//init hardware
//               set_en_high		; force data read from LCD controller
                 digitalWrite(HW_LCD_RS_PIN,LOW);
                 pinMode(HW_LCD_RS_PIN,OUTPUT);
                 digitalWrite(HW_LCD_EN_PIN,HIGH);
   #else // Here for 144 Green Tab, 4 Lines ( RS Line )

   // SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
    
//             sbrc    preg_1, 0
//                     set_rs_high
               if ((preg_1&1)==1) digitalWrite(HW_LCD_RS_PIN,HIGH);
//             sbrs    preg_1, 0
//                     set_rs_low
               if ((preg_1&1)==0) digitalWrite(HW_LCD_RS_PIN,LOW);
               //set_rs_output;		//init hardware
               pinMode(HW_LCD_RS_PIN,OUTPUT);
               //set_b0_output		; wait for address setup, set B0 to output
               pinMode(HW_LCD_B0_PIN,OUTPUT);
   #endif

  
   //SPI.setBitOrder(MSBFIRST);
   //SPI.transfer(preg_2); This will kill the operation ( software incompatible with SPI ressources )
  //SPI.endTransaction;
   
               //; Send bit-7
               //set_en_low
               digitalWrite(HW_LCD_EN_PIN,LOW);
               //sbrc    preg_2, 7
               //set_b0_high
               if ((preg_2&128)==128) digitalWrite(HW_LCD_B0_PIN,HIGH);
               //sbrs    preg_2, 7
               //set_b0_low
               if ((preg_2&128)==0) digitalWrite(HW_LCD_B0_PIN,LOW);
               //set_en_high		; force data read from LCD controller
               digitalWrite(HW_LCD_EN_PIN,HIGH);
               
               //; Send bit-6
               //set_en_low
               //sbrc    preg_2, 6
               //         set_b0_high
               //sbrs    preg_2, 6
               //         set_b0_low
               //set_en_high		; force data read from LCD controller
               digitalWrite(HW_LCD_EN_PIN,LOW);
               if ((preg_2&64)==64) digitalWrite(HW_LCD_B0_PIN,HIGH);
               if ((preg_2&64)==0) digitalWrite(HW_LCD_B0_PIN,LOW);
               digitalWrite(HW_LCD_EN_PIN,HIGH);

               //; Send bit-5
               //set_en_low
               //sbrc    preg_2, 5
               //         set_b0_high
               //sbrs    preg_2, 5
               //        set_b0_low
               //set_en_high		; force data read from LCD controller
               digitalWrite(HW_LCD_EN_PIN,LOW);
               if ((preg_2&32)==32) digitalWrite(HW_LCD_B0_PIN,HIGH);
               if ((preg_2&32)==0) digitalWrite(HW_LCD_B0_PIN,LOW);
               digitalWrite(HW_LCD_EN_PIN,HIGH);

               //; Send bit-4
               //set_en_low
               //sbrc    preg_2, 4
               //       set_b0_high
               //sbrs    preg_2, 4
               //       set_b0_low
               //set_en_high		; force data read from LCD controller
               digitalWrite(HW_LCD_EN_PIN,LOW);
               if ((preg_2&16)==16) digitalWrite(HW_LCD_B0_PIN,HIGH);
               if ((preg_2&16)==0) digitalWrite(HW_LCD_B0_PIN,LOW);
               digitalWrite(HW_LCD_EN_PIN,HIGH);

               //; Send bit-3
               //set_en_low
               //sbrc    preg_2, 3
               //       set_b0_high
               //sbrs    preg_2, 3
               //       set_b0_low
               //set_en_high		; force data read from LCD controller
               digitalWrite(HW_LCD_EN_PIN,LOW);
               if ((preg_2&8)==8) digitalWrite(HW_LCD_B0_PIN,HIGH);
               if ((preg_2&8)==0) digitalWrite(HW_LCD_B0_PIN,LOW);
               digitalWrite(HW_LCD_EN_PIN,HIGH);

               //; Send bit-2
               //set_en_low
               //sbrc    preg_2, 2
               //       set_b0_high
               //sbrs    preg_2, 2
               //       set_b0_low
               //set_en_high		; force data read from LCD controller
               digitalWrite(HW_LCD_EN_PIN,LOW);
               if ((preg_2&4)==4) digitalWrite(HW_LCD_B0_PIN,HIGH);
               if ((preg_2&4)==0) digitalWrite(HW_LCD_B0_PIN,LOW);
               digitalWrite(HW_LCD_EN_PIN,HIGH);

               //; Send bit-1
               //set_en_low
               //sbrc    preg_2, 1
               //       set_b0_high
               //sbrs    preg_2, 1
               //       set_b0_low
               //set_en_high              ; force data read from LCD controller
               digitalWrite(HW_LCD_EN_PIN,LOW);
               if ((preg_2&2)==2) digitalWrite(HW_LCD_B0_PIN,HIGH);
               if ((preg_2&2)==0) digitalWrite(HW_LCD_B0_PIN,LOW);
               digitalWrite(HW_LCD_EN_PIN,HIGH);
               
               //; Send bit-0
               //set_en_low
               //sbrc    preg_2, 0
               //       set_b0_high
               //sbrs    preg_2, 0
               //       set_b0_low
               //set_en_high              ; force data read from LCD controller
               digitalWrite(HW_LCD_EN_PIN,LOW);
               if ((preg_2&1)==1) digitalWrite(HW_LCD_B0_PIN,HIGH);
               if ((preg_2&1)==0) digitalWrite(HW_LCD_B0_PIN,LOW);
               digitalWrite(HW_LCD_EN_PIN,HIGH);
               
   #if (LCD_INTERFACE_MODE == MODE_3LINE)
               //set_rs_high		; SCE to VCC
               digitalWrite(HW_LCD_RS_PIN,HIGH);
   #endif
  //set_ce_high		// disable chip
  //set_en_low
  digitalWrite(HW_LCD_CE_PIN,HIGH);
  digitalWrite(HW_LCD_EN_PIN,LOW);
	//set_b0_low		// ## reset b0 to GND to prevent incorrect detection of rotary encoder movement
	//ret		// return _lcd_hw_write
  digitalWrite(HW_LCD_B0_PIN,LOW);
  }
  #else // no FAST_SPI_OUTPUT 
               //; Set RS (0=Cmd, 1=Char)
        //set_ce_low		// enable chip
	//set_ce_output
	digitalWrite(HW_LCD_CE_PIN,LOW);
	pinMode(HW_LCD_CE_PIN,OUTPUT);
   #if (LCD_INTERFACE_MODE == MODE_3LINE)
               //set_en_low
               //set_rs_low
               //set_rs_output		//init hardware
               digitalWrite(HW_LCD_EN_PIN,LOW);
               digitalWrite(HW_LCD_RS_PIN,LOW);
	             pinMode(HW_LCD_RS_PIN,OUTPUT);
               //sbrc    preg_1, 0
               //       set_b0_high	// set to data
               //sbrs    preg_1, 0
               //      set_b0_low	// set to command
               if ((preg_1&1)==1) digitalWrite(HW_LCD_B0_PIN,HIGH);
               if ((preg_1&1)==0) digitalWrite(HW_LCD_B0_PIN,LOW);
               //set_b0_output		; set B0 to output
               //set_rs_low		; SCE to GND
               //set_en_high		; force data read from LCD controller
                pinMode(HW_LCD_B0_PIN,OUTPUT);
                digitalWrite(HW_LCD_RS_PIN,LOW);
                digitalWrite(HW_LCD_EN_PIN,HIGH);
   #else
               //sbrc    preg_1, 0
               //       set_rs_high
               //sbrs    preg_1, 0
               //      set_rs_low
               if ((preg_1&1)==1) digitalWrite(HW_LCD_RS_PIN,HIGH);
               if ((preg_1&1)==0) digitalWrite(HW_LCD_RS_PIN,LOW);
               //set_rs_output;		//init hardware
               //set_b0_output		; wait for address setup, set B0 to output
               pinMode(HW_LCD_RS_PIN,OUTPUT);
               pinMode(HW_LCD_B0_PIN,OUTPUT);
   #endif
	//; Send bit-7
  //      ROL	preg_2		// shift B7 to carry
  //      rcall	shift_out2
  shift_out2(preg_2 & 0x80);
  
	//; Send bit-6
  //      ROL	preg_2		// shift B6 to carry
  //      rcall	shift_out2
	shift_out2(preg_2 & 0x40);
	
	//; Send bit-5
  //      ROL	preg_2		// shift B5 to carry
  //      rcall	shift_out2
	shift_out2(preg_2 & 0x20);
	
	//; Send bit-4
  //      ROL	preg_2		// shift B4 to carry
  //      rcall	shift_out2
	shift_out2(preg_2 & 0x10);
	
	//; Send bit-3
  //      ROL	preg_2		// shift B3 to carry
  //      rcall	shift_out2
	shift_out2(preg_2 & 0x08);
	
	//; Send bit-2
  //      ROL	preg_2		// shift B2 to carry
  //      rcall	shift_out2
	shift_out2(preg_2 & 0x04);
	
	//; Send bit-1
  //      ROL	preg_2		// shift B1 to carry
  //      rcall	shift_out2
	shift_out2(preg_2 & 0x02);
	
	//; Send bit-0
  //      ROL	preg_2		// shift B0 to carry
  //      rcall	shift_out2
  shift_out2(preg_2 & 0x01);
  
   #if (LCD_INTERFACE_MODE == MODE_3LINE)
        //set_rs_high		; SCE to VCC
        digitalWrite(HW_LCD_RS_PIN,HIGH);
   #endif
    //    set_ce_high		// disable chip
	//set_en_low
	//set_b0_low		// ## reset b0 to GND to prevent incorrect detection of rotary encoder movement
	digitalWrite(HW_LCD_CE_PIN,HIGH);
	digitalWrite(HW_LCD_EN_PIN,LOW);
	digitalWrite(HW_LCD_B0_PIN,LOW);
	//ret		// return _lcd_hw_write
  }
  
  
//shift_out2:
//	set_en_low;
//        brcs	set_bit
//	set_b0_low
//	rjmp	fini_bit
//set_bit:
//	set_b0_high		// enable B0 pullup
//fini_bit:
//	set_b0_output		// set B0 to output mode
//	set_en_high		// set en up
//	ret
	
void shift_out2(unsigned char rol_byte){
//	set_en_low
digitalWrite(HW_LCD_EN_PIN,LOW);
if (rol_byte==0) digitalWrite(HW_LCD_B0_PIN,LOW);
else  digitalWrite(HW_LCD_B0_PIN,HIGH); 
pinMode(HW_LCD_B0_PIN,OUTPUT);
digitalWrite(HW_LCD_EN_PIN,HIGH); 
}		
	
	
  #endif	// FAST_SPI_OUTPUT 
 #endif  // LCD_SPI_OPEN_COL 
//	.endfunc


#elif (LCD_INTERFACE_MODE == MODE_7920_SERIAL) || (LCD_INTERFACE_MODE == MODE_1803_SERIAL)
//	set_b0_high
void _lcd_hw_write(unsigned char preg_1, unsigned char preg_2){
// _lcd_hw_write:
//; 1-bit interface for ST7920 controller

//	set_b0_output		; enable output mode
//	set_en_low
//	set_en_output		; enable output mode
 digitalWrite(HW_LCD_B0_PIN,HIGH);
 pinMode(HW_LCD_B0_PIN,OUTPUT);
 digitalWrite(HW_LCD_EN_PIN,LOW);
 pinMode(HW_LCD_EN_PIN,OUTPUT);
//
//	RCALL	toggle_en	; set en high and low
 void toogle_en();
//
//  RCALL	four_bits	; output four times 1
 void four_bits();
 
//	set_b0_low		; RW to write
//	RCALL	toggle_en	; set en high and low
 digitalWrite(HW_LCD_B0_PIN,LOW);
 void toogle_en(); 

//	sbrc    preg_1, 0
//	set_b0_high		; data mode
 if ((preg_1&1)==1) digitalWrite(HW_LCD_B0_PIN,HIGH);

//	sbrs    preg_1, 0
//	set_b0_low		; instruction mode
//	RCALL	toggle_en	; set en high and low
 if ((preg_1&1)==0) digitalWrite(HW_LCD_B0_PIN,LOW);
 void toogle_en();
//
//	set_b0_low
//	RCALL	toggle_en	; set en high and low
//				; first 8 bit transfer finished
 digitalWrite(HW_LCD_B0_PIN,LOW);
 void toogle_en();
 
#if (LCD_INTERFACE_MODE == MODE_7920_SERIAL)
//  ; output highest bit first
//        sbrc    preg_2, 7
//	 set_b0_high		; bit 7 == 1
//	RCALL	toggle_en	; set en high and low
 if ((preg_2&128)==128) digitalWrite(HW_LCD_B0_PIN,HIGH);
 void toogle_en();
 
//	set_b0_low
//        sbrc    preg_2, 6
//	 set_b0_high		; bit 6 == 1
//	RCALL	toggle_en	; set en high and low
 digitalWrite(HW_LCD_B0_PIN,LOW);
 if ((preg_2&64)==64) digitalWrite(HW_LCD_B0_PIN,HIGH);
 void toogle_en();

//	set_b0_low
//        sbrc    preg_2, 5
//	 set_b0_high		; bit 5 == 1
//	RCALL	toggle_en	; set en high and low
 digitalWrite(HW_LCD_B0_PIN,LOW);
 if ((preg_2&32)==32) digitalWrite(HW_LCD_B0_PIN,HIGH);
 void toogle_en();
 
//
//	set_b0_low
//        sbrc    preg_2, 4
//	 set_b0_high		; bit 4 == 1
//	RCALL	toggle_en	; set en high and low
 digitalWrite(HW_LCD_B0_PIN,LOW);
 if ((preg_2&16)==16) digitalWrite(HW_LCD_B0_PIN,HIGH);
 void toogle_en();

//	set_b0_low
//	RCALL	four_bits	; output 4 times 0
//				; the upper 4-bit are followed by 4 x 0
//	set_b0_low
//        sbrc    preg_2, 3
//	 set_b0_high		; bit 3 == 1
//	RCALL	toggle_en	; set en high and low
 digitalWrite(HW_LCD_B0_PIN,LOW);
 void four_bits();
 digitalWrite(HW_LCD_B0_PIN,LOW);
 if ((preg_2&8)==8) digitalWrite(HW_LCD_B0_PIN,HIGH);
 void toogle_en();

//	set_b0_low
//        sbrc    preg_2, 2
//	 set_b0_high		; bit 2 == 1
//	RCALL	toggle_en	; set en high and low
 digitalWrite(HW_LCD_B0_PIN,LOW);
 if ((preg_2&4)==4) digitalWrite(HW_LCD_B0_PIN,HIGH);
 void toogle_en();

//	set_b0_low
//        sbrc    preg_2, 1
//	 set_b0_high		; bit 1 == 1
//	RCALL	toggle_en	; set en high and low
 digitalWrite(HW_LCD_B0_PIN,LOW);
 if ((preg_2&2)==2) digitalWrite(HW_LCD_B0_PIN,HIGH);
 void toogle_en();

//	set_b0_low
//        sbrc    preg_2, 0
//	 set_b0_high		; bit 0 == 1
//	RCALL	toggle_en	; set en high and low
 digitalWrite(HW_LCD_B0_PIN,LOW);
 if ((preg_2&1)==1) digitalWrite(HW_LCD_B0_PIN,HIGH);
 void toogle_en();

//	set_b0_low
//	RCALL	four_bits	; output 4 times 0
//				; the lower 4-bit are followed by 4 x 0
 digitalWrite(HW_LCD_B0_PIN,LOW);
 void four_bits();
 
#else		/* (LCD_INTERFACE_MODE == MODE_1803_SERIAL) */
//; output lowest bit first
//        sbrc    preg_2, 0
//	 set_b0_high		; bit 0 == 1
//	RCALL	toggle_en	; set en high and low
 if ((preg_2&1)==1) digitalWrite(HW_LCD_B0_PIN,HIGH);
 void toogle_en();

//	set_b0_low
//        sbrc    preg_2, 1
//	 set_b0_high		; bit 1 == 1
//	RCALL	toggle_en	; set en high and low
 digitalWrite(HW_LCD_B0_PIN,LOW);
 if ((preg_2&2)==2) digitalWrite(HW_LCD_B0_PIN,HIGH);
 void toogle_en();

//	set_b0_low
//        sbrc    preg_2, 2
//	 set_b0_high		; bit 2 == 1
//	RCALL	toggle_en	; set en high and low
//				; the upper 4-bit are followed by 4 x 0
digitalWrite(HW_LCD_B0_PIN,LOW);
 if ((preg_2&4)==4) digitalWrite(HW_LCD_B0_PIN,HIGH);
 void toogle_en();

//	set_b0_low
//        sbrc    preg_2, 3
//	 set_b0_high		; bit 3 == 1
//	RCALL	toggle_en	; set en high and low
 digitalWrite(HW_LCD_B0_PIN,LOW);
 if ((preg_2&8)==8) digitalWrite(HW_LCD_B0_PIN,HIGH);
 void toogle_en();

//	set_b0_low
//	RCALL	four_bits//	; output 4 times 0
//				; the lowe//r 4-bit are followed by 4 x 0
digitalWrite(HW_LCD_B0_PIN,LOW);
 void four_bits();

//	set_b0_low
//        sbrc    preg_2, 4
//	 set_b0_high		; bit 4 == 1
//	RCALL	toggle_en	; set en high and low
digitalWrite(HW_LCD_B0_PIN,LOW);
 if ((preg_2&16)==16) digitalWrite(HW_LCD_B0_PIN,HIGH);
 void toogle_en();

//	set_b0_low
//        sbrc    preg_2, 5
//	 set_b0_high		; bit 5 == 1
//	RCALL	toggle_en	; set en high and low
digitalWrite(HW_LCD_B0_PIN,LOW);
 if ((preg_2&32)==32) digitalWrite(HW_LCD_B0_PIN,HIGH);
 void toogle_en();

//	set_b0_low
//        sbrc    preg_2, 6
//	 set_b0_high		; bit 6 == 1
//	RCALL	toggle_en	; set en high and low
 digitalWrite(HW_LCD_B0_PIN,LOW);
 if ((preg_2&64)==64) digitalWrite(HW_LCD_B0_PIN,HIGH);
 void toogle_en();

//	set_b0_low
//        sbrc    preg_2, 7
//	 set_b0_high		; bit 7 == 1
//	RCALL	toggle_en	; set en high and low
 digitalWrite(HW_LCD_B0_PIN,LOW);
 if ((preg_2&128)==128) digitalWrite(HW_LCD_B0_PIN,HIGH);
 void toogle_en();

//	set_b0_low
//	RCALL	four_bits	; output 4 times 0
//				; the upper 4-bit are followed by 4 x 0
	digitalWrite(HW_LCD_B0_PIN,LOW);
 void four_bits();			
#endif
//	RCALL	wait50us
//	RCALL	wait30us	; at least 72 us delay
//	ret		// return _lcd_hw_write
//	.endfunc
 void wait100us();
}

//toggle_en:
//	set_en_high              ;force data read from LCD controller
//        set_en_high		; hold en high to meet the specification (300ns)
//	set_en_low		; set SCLK back to low
//	ret
void toggle_en(){
 digitalWrite(HW_LCD_EN_PIN,HIGH);
 digitalWrite(HW_LCD_EN_PIN,HIGH);
 digitalWrite(HW_LCD_EN_PIN,LOW);
}

/* output 4 times the same bit */
//four_bits:
//	RCALL toggle_en
//	RCALL toggle_en
//	RCALL toggle_en
//	RCALL toggle_en
//	ret
void four_bits(){
  void toogle_en();
  void toogle_en();
  void toogle_en();
  void toogle_en();
}

 
#elif (LCD_INTERFACE_MODE == MODE_I2C)

void _lcd_hw_write(unsigned char preg_1, unsigned char preg_2){

// I2C working, tested with OLED Kuman KY34 , use Arduino dedicated I2C Pins SDA,SCL with full speed
 Wire.beginTransmission(LCD_I2C_ADDR);
 if (preg_1==0) Wire.write(0x80); else Wire.write(0x40); // i2C send Command flag or Data flag
 //if (preg_1!=0) Wire.write(0x40); // Data flag
 Wire.write(preg_2); // Command or Data content
 Wire.endTransmission();
}

void i2c_init(){
  Wire.begin();
}

/*  This not working yet

unsigned char save_preg_1, save_preg_2;
//_lcd_hw_write:
//	; use I2C as master
//	push 	preg_2
//        push	preg_1		; save data/command
//	release_scl
//	rcall	WAIT_I2C
//	set_low_sda		; set START bit
//	rcall	WAIT_I2C
save_preg_2=preg_2;
save_preg_1=preg_1;
pinMode(HW_LCD_SCL_PIN,INPUT_PULLUP);
void WAIT_I2C();
pinMode(HW_LCD_SDA_PIN,OUTPUT);
digitalWrite(HW_LCD_SDA_PIN,LOW);
void WAIT_I2C();
//	ldi	preg_1, (LCD_I2C_ADDR*2)
//	rcall	i2c_send	; write I2C address
preg_1=(LCD_I2C_ADDR*2);
i2c_send(preg_1,preg_2);
//	pop	preg_2
//	ldi	preg_1,0x80	; send command type
preg_1=save_preg_1;
//	sbrc	preg_2,0	; skip if bit 0 is unset
//	ldi	preg_1,0x40	; send data type
if (preg_1==0) preg_1=0x80; else preg_1=0x40;
//	rcall	i2c_send	; send command/data
//	pop	preg_1		;restore data from parameter
//	rcall	i2c_send	; write the data
 i2c_send(preg_1,preg_2);
preg_1=save_preg_2;
 i2c_send(preg_1,preg_2);
//	set_low_sda		; set the sda signal to low STOP
//	rcall	WAIT_I2C
//	release_scl		; pullup move the scl signal to high
//	rcall	WAIT_I2C
//	release_sda		; pullup move the sda signal to high, STOP
//	rcall	WAIT_I2C
pinMode(HW_LCD_SDA_PIN,OUTPUT);
digitalWrite(HW_LCD_SDA_PIN,LOW);
void WAIT_I2C();
pinMode(HW_LCD_SCL_PIN,INPUT_PULLUP);
void WAIT_I2C();
//pinMode(HW_LCD_SDA_PIN,INPUT_PULLUP);
void WAIT_I2C();
//	ret		// return _lcd_hw_write
}
//;
//;===================================================
//i2c_send:
void i2c_send(unsigned char preg_1, unsigned char preg_2){
//	sec			;set carry
//	rol	preg_1		; shift carry to r24 bit 0 and bit 7 of r24 to carry
//  i2c_wf:
//	set_low_scl		; scl signal to low, data change
//	brcc	wr0
//			; carry was set
//	release_sda		; pullup move the sda signal to high
//	rjmp	wr1
//wr0:
//	set_low_sda		; set the sda signal to low
//wr1:
for ( unsigned char i=128 ; i!=0 ; i=i>>1) {
 digitalWrite(HW_LCD_SCL_PIN,LOW);
 pinMode(HW_LCD_SCL_PIN,OUTPUT);
 if ((preg_1&i)==i) pinMode(HW_LCD_SDA_PIN,INPUT_PULLUP);
 if ((preg_1&i)==0) { pinMode(HW_LCD_SDA_PIN,OUTPUT);digitalWrite(HW_LCD_SDA_PIN,LOW);}
//	rcall	WAIT_I2C	; wait defined time
//	release_scl		; pullup move the scl signal to high
//	rcall	WAIT_I2C	; wait defined time
void WAIT_I2C();
pinMode(HW_LCD_SCL_PIN,INPUT_PULLUP);
void WAIT_I2C();
//	lsl	preg_1
//	brne	i2c_wf
}
//; 8 bit are transfered
//	set_low_scl		; scl signal to low, data change
//	release_sda		; give sda free
//	rcall	WAIT_I2C	; wait defined time
//	release_scl		; pullup move the scl signal to high, ack cycle
digitalWrite(HW_LCD_SCL_PIN,LOW);
pinMode(HW_LCD_SCL_PIN,OUTPUT);
pinMode(HW_LCD_SDA_PIN,INPUT_PULLUP);
void WAIT_I2C();
pinMode(HW_LCD_SCL_PIN,INPUT_PULLUP);
//loop:
//	sbis	HW_LCD_SCL_IN, HW_LCD_SCL_PIN
//	rjmp	loop		; wait for releasing SCL
//; r24 is zero, return 0
while (digitalRead(HW_LCD_SCL_PIN)==LOW){}
preg_1=0;// return 0
//	sbic	HW_LCD_SDA_IN, HW_LCD_SDA_PIN
//	ldi	preg_1,1		; if SDA is returned high, answer 1
if(digitalRead(HW_LCD_SDA_PIN)==HIGH) preg_1=1;// return value 1
//	rcall	WAIT_I2C	; wait defined time
//	set_low_scl
//	rcall	WAIT_I2C	; wait defined time
void WAIT_I2C();
digitalWrite(HW_LCD_SCL_PIN,LOW);
pinMode(HW_LCD_SCL_PIN,OUTPUT);
void WAIT_I2C();
//	ret
//	.endfunc
}
//       .global	i2c_init
//       .func	i2c_init
//       .extern	wait5us
//i2c_init:
//	release_sda
//	release_scl
//	cbi	HW_LCD_SDA_OUT, HW_LCD_SDA_PIN	; set output to 0, no pull up
//	cbi	HW_LCD_SCL_OUT, HW_LCD_SCL_PIN	; set output to 0, no pull up
//
//	ret
//	.endfunc

void i2c_init(){
  pinMode(HW_LCD_SDA_PIN,INPUT_PULLUP);
  pinMode(HW_LCD_SCL_PIN,INPUT_PULLUP);
  digitalWrite(HW_LCD_SDA_PIN,LOW);
  digitalWrite(HW_LCD_SCL_PIN,LOW);
  pinMode(HW_LCD_SDA_PIN,OUTPUT);
  pinMode(HW_LCD_SCL_PIN,OUTPUT);
}
*/



#elif (LCD_INTERFACE_MODE == MODE_7108_SERIAL)
void _lcd_hw_write(unsigned char preg_1, unsigned char preg_2){
//_lcd_hw_write:
//; serial interface for ST7108 controller
//	set_clk_low
//	set_clk_output
digitalWrite(HW_LCD_CLK_PIN,LOW);
pinMode(HW_LCD_CLK_PIN,OUTPUT);
//	set_pclk_low
//	set_pclk_output

digitalWrite(HW_LCD_PCLK_PIN,LOW);
pinMode(HW_LCD_PCLK_PIN,OUTPUT);
//        set_en_low
//        set_en_output
digitalWrite(HW_LCD_EN_PIN,LOW);
pinMode(HW_LCD_EN_PIN,OUTPUT);
//	set_b0_low
//        set_b0_output
digitalWrite(HW_LCD_B0_PIN,LOW);
pinMode(HW_LCD_B0_PIN,OUTPUT);
//        sbrc    preg_2, 7
//	 set_b0_high		; bit 7 == 1
if ((preg_2&128)==128) digitalWrite(HW_LCD_B0_PIN,HIGH);	 
 #ifdef FAST_SERIAL_OUT
	//set_clk_high		; set clk high and low
	//set_clk_low
	//set_b0_low
	digitalWrite(HW_LCD_CLK_PIN,HIGH);
	digitalWrite(HW_LCD_CLK_PIN,LOW);
	digitalWrite(HW_LCD_B0_PIN,LOW);
 #else
	//RCALL	toggle_clk	; set clk high and low
	void toggle_clk();
 #endif

//        sbrc    preg_2, 6
//	 set_b0_high		; bit 6 == 1
// #ifdef FAST_SERIAL_OUT
//	set_clk_high		; set clk high and low
//	set_clk_low
//	set_b0_low
// #else
//	RCALL	toggle_clk	; set clk high and low
// #endif
if ((preg_2&64)==64) digitalWrite(HW_LCD_B0_PIN,HIGH);	 
 #ifdef FAST_SERIAL_OUT
	digitalWrite(HW_LCD_CLK_PIN,HIGH);
	digitalWrite(HW_LCD_CLK_PIN,LOW);
	digitalWrite(HW_LCD_B0_PIN,LOW);
 #else
	void toggle_clk();
 #endif

//        sbrc    preg_2, 5
//	 set_b0_high		; bit 5 == 1
// #ifdef FAST_SERIAL_OUT
//	set_clk_high		; set clk high and low
//	set_clk_low
//	set_b0_low
// #else
//	RCALL	toggle_clk	; set clk high and low
// #endif
if ((preg_2&32)==32) digitalWrite(HW_LCD_B0_PIN,HIGH);	 
 #ifdef FAST_SERIAL_OUT
	digitalWrite(HW_LCD_CLK_PIN,HIGH);
	digitalWrite(HW_LCD_CLK_PIN,LOW);
	digitalWrite(HW_LCD_B0_PIN,LOW);
 #else
	void toggle_clk();
 #endif

//        sbrc    preg_2, 4
//	 set_b0_high		; bit 4 == 1
// #ifdef FAST_SERIAL_OUT
//	set_clk_high		; set clk high and low
//	set_clk_low
//	set_b0_low
// #else
//	RCALL	toggle_clk	; set clk high and low
// #endif
if ((preg_2&16)==16) digitalWrite(HW_LCD_B0_PIN,HIGH);	 
 #ifdef FAST_SERIAL_OUT
	digitalWrite(HW_LCD_CLK_PIN,HIGH);
	digitalWrite(HW_LCD_CLK_PIN,LOW);
	digitalWrite(HW_LCD_B0_PIN,LOW);
 #else
	void toggle_clk();
 #endif

//        sbrc    preg_2, 3
//	 set_b0_high		; bit 3 == 1
// #ifdef FAST_SERIAL_OUT
//	set_clk_high		; set clk high and low
//	set_clk_low
//	set_b0_low
// #else
//	RCALL	toggle_clk	; set clk high and low
// #endif
if ((preg_2&8)==8) digitalWrite(HW_LCD_B0_PIN,HIGH);	 
 #ifdef FAST_SERIAL_OUT
	digitalWrite(HW_LCD_CLK_PIN,HIGH);
	digitalWrite(HW_LCD_CLK_PIN,LOW);
	digitalWrite(HW_LCD_B0_PIN,LOW);
 #else
	void toggle_clk();
 #endif

//        sbrc    preg_2, 2
//	 set_b0_high		; bit 2 == 1
// #ifdef FAST_SERIAL_OUT
//	set_clk_high		; set clk high and low
//	set_clk_low
//	set_b0_low
// #else
//	RCALL	toggle_clk	; set clk high and low
// #endif
if ((preg_2&4)==4) digitalWrite(HW_LCD_B0_PIN,HIGH);	 
 #ifdef FAST_SERIAL_OUT
	digitalWrite(HW_LCD_CLK_PIN,HIGH);
	digitalWrite(HW_LCD_CLK_PIN,LOW);
	digitalWrite(HW_LCD_B0_PIN,LOW);
 #else
	void toggle_clk();
 #endif

//        sbrc    preg_2, 1
//	 set_b0_high		; bit 1 == 1
// #ifdef FAST_SERIAL_OUT
//	set_clk_high		; set clk high and low
//	set_clk_low
//	set_b0_low
// #else
//	RCALL	toggle_clk	; set clk high and low
// #endif
if ((preg_2&2)==2) digitalWrite(HW_LCD_B0_PIN,HIGH);	 
 #ifdef FAST_SERIAL_OUT
	digitalWrite(HW_LCD_CLK_PIN,HIGH);
	digitalWrite(HW_LCD_CLK_PIN,LOW);
	digitalWrite(HW_LCD_B0_PIN,LOW);
 #else
	void toggle_clk();
 #endif

//        sbrc    preg_2, 0
//	 set_b0_high		; bit 0 == 1
// #ifdef FAST_SERIAL_OUT
//	set_clk_high		; set clk high and low
//	set_clk_low
//	set_b0_low
// #else
//	RCALL	toggle_clk	; set clk high and low
// #endif
if ((preg_2&1)==1) digitalWrite(HW_LCD_B0_PIN,HIGH);	 
 #ifdef FAST_SERIAL_OUT
	digitalWrite(HW_LCD_CLK_PIN,HIGH);
	digitalWrite(HW_LCD_CLK_PIN,LOW);
	digitalWrite(HW_LCD_B0_PIN,LOW);
 #else
	void toggle_clk();
 #endif
 
//        ; all 8 bit are loaded to the 74HC164 output
//	set_pclk_high		; set parallel clk high and low
//	set_pclk_low
	digitalWrite(HW_LCD_PCLK_PIN,HIGH);
  digitalWrite(HW_LCD_PCLK_PIN,LOW);

//	set_rs_low		; instruction mode
//        set_rs_output		; if RS is set to same as B0, RS is allready output
digitalWrite(HW_LCD_RS_PIN,LOW);
digitalWrite(HW_LCD_RS_PIN,HIGH);
//	sbrc    preg_1, 0
//	set_rs_high		; data mode
if ((preg_1&1)==1) digitalWrite(HW_LCD_RS_PIN,HIGH);	 
//	RCALL	wait1us		; hold the setup time of RS
//	set_en_high
//	RCALL	wait1us
//	set_en_low
	digitalWrite(HW_LCD_EN_PIN,HIGH);
	digitalWrite(HW_LCD_EN_PIN,LOW);	
	
//;	RCALL	wait30us	; at least 30 us delay
//	RCALL	wait10us	; at least 10 us delay
void wait10us();
//	ret
//	.endfunc
}
 #ifndef FAST_SERIAL_OUT
//toggle_clk:
//	set_clk_high
//	set_clk_high
//	set_clk_low
//	set_b0_low
//	ret
void toggle_clk(){
  	digitalWrite(HW_LCD_CLK_PIN,HIGH);
  	digitalWrite(HW_LCD_CLK_PIN,HIGH);
  	digitalWrite(HW_LCD_CLK_PIN,LOW);
  	digitalWrite(HW_LCD_B0_PIN,LOW);
}
 #endif


#else	/* !(LCD_INTERFACE_MODE == (MODE_SPI | MODE_7920_SERIAL | MODE_I2C | MODE_7108_SERIAL)) */
// 4 Bit interface HD44780

// with I2C interface at 0x27

#ifdef WITH_1602_I2C
void _lcd_hw_write(unsigned char preg_1, unsigned char preg_2){
lcd_send(preg_2,preg_1);
}

void lcd_send(uint8_t value, uint8_t mode) {
  uint8_t highnib=value&0xf0;
  uint8_t lownib=(value<<4)&0xf0;
  write4bits((highnib)|mode);
  write4bits((lownib)|mode);
}


void write4bits(uint8_t value) {
  expanderWrite(value);
  pulseEnable(value);
}

void expanderWrite(uint8_t _data){
  Wire.beginTransmission(LCD_I2C_ADDR); //_addr
  Wire.write((int)(_data) | 0x08); //_backlightval);
  Wire.endTransmission();
}

void pulseEnable(uint8_t _data){
  #define En B00000100  // Enable bit

  expanderWrite(_data | En);  // En high
  delayMicroseconds(1);   // enable pulse must be >450ns

  expanderWrite(_data & ~En); // En low
  delayMicroseconds(50);    // commands need > 37us to settle
}

#else  // paralell no I2C

void _lcd_hw_write(unsigned char preg_1, unsigned char preg_2){

//sbrc    preg_1, 0
//        set_rs_high
//sbrs    preg_1, 0
//        set_rs_low
//
 if ((preg_1&1)==1) digitalWrite(HW_LCD_RS_PIN,HIGH);//RS
 if ((preg_1&1)==0) digitalWrite(HW_LCD_RS_PIN,LOW);
//set_rs_output;		//init hardware
//set_en_high
//set_en_output;		//init hardware
 pinMode(HW_LCD_RS_PIN,OUTPUT);
 digitalWrite(HW_LCD_EN_PIN,HIGH);//EN
 pinMode(HW_LCD_EN_PIN,OUTPUT);
//Send high nibble
//set_b4_low
//set_b5_low
//set_b6_low
//set_b7_low
 digitalWrite(HW_LCD_B4_PIN,LOW);//B4  
 digitalWrite(HW_LCD_B5_PIN,LOW);//B5 
 digitalWrite(HW_LCD_B6_PIN,LOW);//B6 
 digitalWrite(HW_LCD_B7_PIN,LOW);//B7 
//sbrc    preg_2, 4
//        set_b4_high
//set_b4_output;		//init hardware
 if ((preg_2&16)==16) digitalWrite(HW_LCD_B4_PIN,HIGH);
 pinMode(HW_LCD_B4_PIN,OUTPUT);//B4
//sbrc    preg_2, 5
//set_b5_high
//set_b5_output;		//init hardware
 if ((preg_2&32)==32) digitalWrite(HW_LCD_B5_PIN,HIGH);
 pinMode(HW_LCD_B5_PIN,OUTPUT);//B5
//sbrc    preg_2, 6
//        set_b6_high
//set_b6_output;		//init hardware
 if ((preg_2&64)==64) digitalWrite(HW_LCD_B6_PIN,HIGH);
 pinMode(HW_LCD_B6_PIN,OUTPUT);//B6
//sbrc    preg_2, 7
//        set_b7_high
//set_b7_output;		//init hardware
 if ((preg_2&128)==128) digitalWrite(HW_LCD_B7_PIN,HIGH);
 pinMode(HW_LCD_B7_PIN,OUTPUT);//b7
//nop			; wait for data setup time
//               set_en_low		; force data read from LCD controller
//               RCALL    wait1us
 digitalWrite(HW_LCD_EN_PIN,LOW);//EN=Low
//skip sending low nibble for init commands
//sbrc    preg_1, 7
//         rjmp _lcd_hw_write_exit
 if ((preg_1&128)==128){}
 else
 {
//Send low nibble
//set_en_high
//set_b4_low
//set_b5_low
//set_b6_low
//set_b7_low
  digitalWrite(HW_LCD_EN_PIN,HIGH);//EN=High
  digitalWrite(HW_LCD_B4_PIN,LOW);//B4=Low
  digitalWrite(HW_LCD_B5_PIN,LOW);//B5=Low
  digitalWrite(HW_LCD_B6_PIN,LOW);//B6=Low
  digitalWrite(HW_LCD_B7_PIN,LOW);//B7=Low
//sbrc    preg_2, 0
//        set_b4_high
  if ((preg_2&1)==1) digitalWrite(HW_LCD_B4_PIN,HIGH);
//sbrc    preg_2, 1
//        set_b5_high
  if ((preg_2&2)==2) digitalWrite(HW_LCD_B5_PIN,HIGH);
//sbrc    preg_2, 2
//        set_b6_high
  if ((preg_2&4)==4) digitalWrite(HW_LCD_B6_PIN,HIGH);
//sbrc    preg_2, 3
//        set_b7_high
  if ((preg_2&8)==8) digitalWrite(HW_LCD_B7_PIN,HIGH); 
//nop			; wait for data setup time
//set_en_low		; force data read from LCD controller
  digitalWrite(HW_LCD_EN_PIN,LOW);//EN=Low
  
 #if (LCD_ST_TYPE == 7920)
               void wait50us();
 #endif
 #ifdef SLOW_LCD
               void wait50us();
 #else
               void wait1us();
 #endif

 }
 
} // ; end _lcd_hw_write
#endif  /* WITH_1602_I2C */

#endif /* LCD_INTERFACE_MODE */


#if (LCD_ST_TYPE == 7108)
//      .global _lcd_hw_select
//     .func _lcd_hw_select
//; select one of the two controllers or both
//;
//;      preg_1 (r24) = bit 0 for CS1 and bit 1 for CS2
//;
//_lcd_hw_select:
void _lcd_hw_select(unsigned char preg_1){
#ifdef ST_CS_LOW   /* inverted CS level, 0 = enable */
//	sbrc    preg_1, 0
//	  set_cs1_low		; enable controller 1
	  if ((preg_1&1)==1) digitalWrite(HW_LCD_CS1_PIN,LOW);
//	sbrs    preg_1, 0
//	  set_cs1_high		; disable controller 1
	  if ((preg_1&1)==0) digitalWrite(HW_LCD_CS1_PIN,HIGH);  
//	sbrc    preg_1, 1
//	  set_cs2_low		; enable controller 2
//	sbrs    preg_1, 1
//	  set_cs2_high		; disable controller 2
    if ((preg_1&2)==2) digitalWrite(HW_LCD_CS2_PIN,LOW);
    if ((preg_1&2)==0) digitalWrite(HW_LCD_CS2_PIN,HIGH);
#else	/* not inverted CS level, 1 = enable */
//	sbrc    preg_1, 0
//	  set_cs1_high		; enable controller 1
//	sbrs    preg_1, 0
//	  set_cs1_low		; disable controller 1
	  if ((preg_1&1)==1) digitalWrite(HW_LCD_CS1_PIN,HIGH);
	  if ((preg_1&1)==0) digitalWrite(HW_LCD_CS1_PIN,LOW);
//	sbrc    preg_1, 1
//	  set_cs2_high		; enable controller 2
//	sbrs    preg_1, 1
//	  set_cs2_low		; disable controller 2
    if ((preg_1&2)==2) digitalWrite(HW_LCD_CS2_PIN,HIGH);
    if ((preg_1&2)==0) digitalWrite(HW_LCD_CS2_PIN,LOW);
#endif
//	set_cs1_output		; enable output CS1
//	set_cs2_output		; enable output CS2
//	ret
//        .endfunc
pinMode(HW_LCD_CS1_PIN,OUTPUT);
pinMode(HW_LCD_CS2_PIN,OUTPUT);   
}
#endif //(LCD_ST_TYPE == 7108)
