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

#if (LCD_INTERFACE_MODE == MODE_SPI) || (LCD_INTERFACE_MODE == MODE_3LINE)

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
#endif
//	RCALL	wait50us
//	RCALL	wait30us	; at least 72 us delay
//	ret		// return _lcd_hw_write
//	.endfunc
 void wait100us();
}


 
#elif (LCD_INTERFACE_MODE == MODE_I2C)
#elif (LCD_INTERFACE_MODE == MODE_7108_SERIAL)

#else	/* !(LCD_INTERFACE_MODE == (MODE_SPI | MODE_7920_SERIAL)) */ERIAL | MODE_I2C | MODE_7108_S
// 4 Bit interface HD44780
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
 }
 
}
#endif