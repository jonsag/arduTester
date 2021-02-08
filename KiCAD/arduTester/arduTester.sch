EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MCU_Module:Arduino_UNO_R3 A1
U 1 1 601EFF84
P 5300 3200
F 0 "A1" H 5300 4381 50  0000 C CNN
F 1 "Arduino_UNO_R3" H 5300 4290 50  0000 C CNN
F 2 "My_Arduino:Arduino_UNO_R3_shield_larger_pads" H 5300 3200 50  0001 C CIN
F 3 "https://www.arduino.cc/en/Main/arduinoBoardUno" H 5300 3200 50  0001 C CNN
	1    5300 3200
	1    0    0    -1  
$EndComp
$Comp
L My_Headers:LCD_HD44780 U1
U 1 1 601F1CE0
P 1950 4100
F 0 "U1" H 2192 4325 50  0000 C CNN
F 1 "LCD_HD44780" H 2192 4234 50  0000 C CNN
F 2 "My_Parts:LCD_16x2_large" H 3000 2400 50  0001 C CNN
F 3 "" H 2200 4200 50  0001 C CNN
	1    1950 4100
	-1   0    0    1   
$EndComp
Wire Wire Line
	4800 2800 1950 2800
Wire Wire Line
	4800 3100 1950 3100
Wire Wire Line
	4800 2900 1950 2900
Wire Wire Line
	1950 3000 4800 3000
Wire Wire Line
	1950 4100 2050 4100
Wire Wire Line
	2050 4100 2050 3700
Wire Wire Line
	2050 3700 1950 3700
Wire Wire Line
	2050 2600 1950 2600
Connection ~ 2050 3700
Wire Wire Line
	2050 2600 2050 3700
Wire Wire Line
	2050 4100 2050 4450
Connection ~ 2050 4100
$Comp
L power:GND #PWR01
U 1 1 601FD870
P 2050 4450
F 0 "#PWR01" H 2050 4200 50  0001 C CNN
F 1 "GND" H 2055 4277 50  0000 C CNN
F 2 "" H 2050 4450 50  0001 C CNN
F 3 "" H 2050 4450 50  0001 C CNN
	1    2050 4450
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR02
U 1 1 601FF298
P 2150 2450
F 0 "#PWR02" H 2150 2300 50  0001 C CNN
F 1 "+5V" H 2165 2623 50  0000 C CNN
F 2 "" H 2150 2450 50  0001 C CNN
F 3 "" H 2150 2450 50  0001 C CNN
	1    2150 2450
	1    0    0    -1  
$EndComp
Wire Wire Line
	1950 4000 2150 4000
Wire Wire Line
	2150 4000 2150 2450
$Comp
L power:+5V #PWR07
U 1 1 601FF7F3
P 5500 1900
F 0 "#PWR07" H 5500 1750 50  0001 C CNN
F 1 "+5V" H 5515 2073 50  0000 C CNN
F 2 "" H 5500 1900 50  0001 C CNN
F 3 "" H 5500 1900 50  0001 C CNN
	1    5500 1900
	1    0    0    -1  
$EndComp
$Comp
L My_Headers:2-pin_power_input_header J2
U 1 1 601FFE60
P 4400 1500
F 0 "J2" H 4363 1633 50  0000 C CNN
F 1 "2-pin_power_input_header" H 4400 1300 50  0001 C CNN
F 2 "My_Headers:2-pin_power_input_header" H 4450 1200 50  0001 C CNN
F 3 "~" H 4400 1500 50  0001 C CNN
	1    4400 1500
	-1   0    0    -1  
$EndComp
Wire Wire Line
	5200 1500 4600 1500
$Comp
L power:GND #PWR06
U 1 1 60203C0A
P 4700 1700
F 0 "#PWR06" H 4700 1450 50  0001 C CNN
F 1 "GND" H 4705 1527 50  0000 C CNN
F 2 "" H 4700 1700 50  0001 C CNN
F 3 "" H 4700 1700 50  0001 C CNN
	1    4700 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 1600 4700 1600
Wire Wire Line
	4700 1600 4700 1700
Wire Wire Line
	5200 1500 5200 2200
$Comp
L Device:R_POT RV1
U 1 1 60204DB7
P 3250 4200
F 0 "RV1" V 3135 4200 50  0000 C CNN
F 1 "10k" V 3044 4200 50  0000 C CNN
F 2 "My_Misc:Potentiometer_Piher_PT-6-V_Vertical_larger_pads" H 3250 4200 50  0001 C CNN
F 3 "~" H 3250 4200 50  0001 C CNN
	1    3250 4200
	0    1    -1   0   
$EndComp
Wire Wire Line
	3250 3900 3250 4050
$Comp
L power:+5V #PWR05
U 1 1 60205D4F
P 3500 4100
F 0 "#PWR05" H 3500 3950 50  0001 C CNN
F 1 "+5V" H 3515 4273 50  0000 C CNN
F 2 "" H 3500 4100 50  0001 C CNN
F 3 "" H 3500 4100 50  0001 C CNN
	1    3500 4100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 602060F7
P 3000 4500
F 0 "#PWR04" H 3000 4250 50  0001 C CNN
F 1 "GND" H 3005 4327 50  0000 C CNN
F 2 "" H 3000 4500 50  0001 C CNN
F 3 "" H 3000 4500 50  0001 C CNN
	1    3000 4500
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 4500 3000 4200
Wire Wire Line
	3000 4200 3100 4200
Wire Wire Line
	3400 4200 3500 4200
Wire Wire Line
	3500 4200 3500 4100
Wire Wire Line
	1950 3900 3250 3900
Wire Wire Line
	1950 3600 2250 3600
Wire Wire Line
	2250 3600 2250 3200
Wire Wire Line
	2250 3200 4800 3200
Wire Wire Line
	1950 3800 2350 3800
Wire Wire Line
	2350 3800 2350 3300
Wire Wire Line
	2350 3300 4800 3300
$Comp
L Device:R R1
U 1 1 6020AC93
P 2400 2700
F 0 "R1" V 2193 2700 50  0000 C CNN
F 1 "220R" V 2284 2700 50  0000 C CNN
F 2 "My_Misc:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal_larger_pads" V 2330 2700 50  0001 C CNN
F 3 "~" H 2400 2700 50  0001 C CNN
	1    2400 2700
	0    1    1    0   
$EndComp
Wire Wire Line
	1950 2700 2250 2700
Wire Wire Line
	2550 2700 2650 2700
Wire Wire Line
	2650 2700 2650 2450
$Comp
L power:+5V #PWR03
U 1 1 6020C954
P 2650 2450
F 0 "#PWR03" H 2650 2300 50  0001 C CNN
F 1 "+5V" H 2665 2623 50  0000 C CNN
F 2 "" H 2650 2450 50  0001 C CNN
F 3 "" H 2650 2450 50  0001 C CNN
	1    2650 2450
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 6020E00C
P 7000 2600
F 0 "R2" V 6793 2600 50  0000 C CNN
F 1 "680R" V 6884 2600 50  0000 C CNN
F 2 "My_Misc:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal_larger_pads" V 6930 2600 50  0001 C CNN
F 3 "~" H 7000 2600 50  0001 C CNN
	1    7000 2600
	0    1    1    0   
$EndComp
$Comp
L Device:R R3
U 1 1 60211932
P 7550 2600
F 0 "R3" V 7343 2600 50  0000 C CNN
F 1 "470k" V 7434 2600 50  0000 C CNN
F 2 "My_Misc:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal_larger_pads" V 7480 2600 50  0001 C CNN
F 3 "~" H 7550 2600 50  0001 C CNN
	1    7550 2600
	0    1    1    0   
$EndComp
$Comp
L Device:R R4
U 1 1 60213C4D
P 8150 2600
F 0 "R4" V 7943 2600 50  0000 C CNN
F 1 "680R" V 8034 2600 50  0000 C CNN
F 2 "My_Misc:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal_larger_pads" V 8080 2600 50  0001 C CNN
F 3 "~" H 8150 2600 50  0001 C CNN
	1    8150 2600
	0    1    1    0   
$EndComp
$Comp
L Device:R R5
U 1 1 60213C53
P 8700 2600
F 0 "R5" V 8493 2600 50  0000 C CNN
F 1 "470k" V 8584 2600 50  0000 C CNN
F 2 "My_Misc:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal_larger_pads" V 8630 2600 50  0001 C CNN
F 3 "~" H 8700 2600 50  0001 C CNN
	1    8700 2600
	0    1    1    0   
$EndComp
$Comp
L Device:R R6
U 1 1 60215664
P 9300 2600
F 0 "R6" V 9093 2600 50  0000 C CNN
F 1 "680R" V 9184 2600 50  0000 C CNN
F 2 "My_Misc:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal_larger_pads" V 9230 2600 50  0001 C CNN
F 3 "~" H 9300 2600 50  0001 C CNN
	1    9300 2600
	0    1    1    0   
$EndComp
$Comp
L Device:R R7
U 1 1 6021566A
P 9850 2600
F 0 "R7" V 9643 2600 50  0000 C CNN
F 1 "470k" V 9734 2600 50  0000 C CNN
F 2 "My_Misc:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal_larger_pads" V 9780 2600 50  0001 C CNN
F 3 "~" H 9850 2600 50  0001 C CNN
	1    9850 2600
	0    1    1    0   
$EndComp
Wire Wire Line
	7150 2600 7300 2600
Wire Wire Line
	8300 2600 8450 2600
Wire Wire Line
	9450 2600 9600 2600
Text GLabel 7300 2300 1    50   Input ~ 0
TP1
Text GLabel 8450 2300 1    50   Input ~ 0
TP2
Text GLabel 9600 2350 1    50   Input ~ 0
TP3
Wire Wire Line
	7300 2600 7300 2300
Connection ~ 7300 2600
Wire Wire Line
	7300 2600 7400 2600
Wire Wire Line
	8450 2600 8450 2300
Connection ~ 8450 2600
Wire Wire Line
	8450 2600 8550 2600
Wire Wire Line
	9600 2600 9600 2350
Connection ~ 9600 2600
Wire Wire Line
	9600 2600 9700 2600
Wire Wire Line
	5800 3200 7300 3200
Wire Wire Line
	7300 3200 7300 2600
Wire Wire Line
	5800 3300 8450 3300
Wire Wire Line
	8450 3300 8450 2600
Wire Wire Line
	5800 3400 9600 3400
Wire Wire Line
	9600 3400 9600 2600
Wire Wire Line
	4800 3900 4700 3900
Wire Wire Line
	4700 3900 4700 5450
Wire Wire Line
	4700 5450 10100 5450
Wire Wire Line
	10100 5450 10100 2600
Wire Wire Line
	10100 2600 10000 2600
Wire Wire Line
	9150 2600 9050 2600
Wire Wire Line
	9050 2600 9050 5550
Wire Wire Line
	9050 5550 4600 5550
Wire Wire Line
	4600 5550 4600 3800
Wire Wire Line
	4600 3800 4800 3800
Wire Wire Line
	4800 3700 4500 3700
Wire Wire Line
	4500 3700 4500 5650
Wire Wire Line
	4500 5650 8950 5650
Wire Wire Line
	8950 5650 8950 2600
Wire Wire Line
	8950 2600 8850 2600
Wire Wire Line
	8000 2600 7900 2600
Wire Wire Line
	7900 2600 7900 5750
Wire Wire Line
	7900 5750 4400 5750
Wire Wire Line
	4400 5750 4400 3600
Wire Wire Line
	4400 3600 4800 3600
Wire Wire Line
	7700 2600 7800 2600
Wire Wire Line
	7800 2600 7800 5850
Wire Wire Line
	7800 5850 4300 5850
Wire Wire Line
	4300 5850 4300 3500
Wire Wire Line
	4300 3500 4800 3500
Wire Wire Line
	4800 3400 4200 3400
Wire Wire Line
	4200 3400 4200 5950
Wire Wire Line
	4200 5950 6750 5950
Wire Wire Line
	6750 5950 6750 2600
Wire Wire Line
	6750 2600 6850 2600
$Comp
L Switch:SW_Push SW1
U 1 1 602482DF
P 6100 4550
F 0 "SW1" V 6054 4698 50  0000 L CNN
F 1 "SW_Push" V 6145 4698 50  0000 L CNN
F 2 "My_Misc:SW_PUSH-12mm_large" H 6100 4750 50  0001 C CNN
F 3 "~" H 6100 4750 50  0001 C CNN
	1    6100 4550
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR09
U 1 1 60248E9E
P 6100 4850
F 0 "#PWR09" H 6100 4600 50  0001 C CNN
F 1 "GND" H 6105 4677 50  0000 C CNN
F 2 "" H 6100 4850 50  0001 C CNN
F 3 "" H 6100 4850 50  0001 C CNN
	1    6100 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 4850 6100 4750
Wire Wire Line
	6100 4350 6100 3500
Wire Wire Line
	6100 3500 5800 3500
$Comp
L power:+5V #PWR08
U 1 1 6024F048
P 6100 1950
F 0 "#PWR08" H 6100 1800 50  0001 C CNN
F 1 "+5V" H 6115 2123 50  0000 C CNN
F 2 "" H 6100 1950 50  0001 C CNN
F 3 "" H 6100 1950 50  0001 C CNN
	1    6100 1950
	1    0    0    -1  
$EndComp
$Comp
L My_Headers:3-pin_test_header J1
U 1 1 60266FF2
P 2050 5700
F 0 "J1" H 2219 5715 50  0000 L CNN
F 1 "3-pin_test_header" H 2050 5500 50  0001 C CNN
F 2 "My_Headers:3-pin_test_header_large" H 2100 5400 50  0001 C CNN
F 3 "~" H 2050 5700 50  0001 C CNN
	1    2050 5700
	1    0    0    -1  
$EndComp
Text GLabel 1850 5600 0    50   Input ~ 0
TP1
Text GLabel 1850 5700 0    50   Input ~ 0
TP2
Text GLabel 1850 5800 0    50   Input ~ 0
TP3
$Comp
L My_Headers:3-pin_test_header J3
U 1 1 601FA8F1
P 2050 6250
F 0 "J3" H 2219 6265 50  0000 L CNN
F 1 "3-pin_test_header" H 2050 6050 50  0001 C CNN
F 2 "My_Headers:3-pin_test_header_large" H 2100 5950 50  0001 C CNN
F 3 "~" H 2050 6250 50  0001 C CNN
	1    2050 6250
	1    0    0    -1  
$EndComp
Text GLabel 1850 6150 0    50   Input ~ 0
TP1
Text GLabel 1850 6250 0    50   Input ~ 0
TP2
Text GLabel 1850 6350 0    50   Input ~ 0
TP3
$Comp
L power:GND #PWR0101
U 1 1 60208A59
P 5200 4400
F 0 "#PWR0101" H 5200 4150 50  0001 C CNN
F 1 "GND" H 5205 4227 50  0000 C CNN
F 2 "" H 5200 4400 50  0001 C CNN
F 3 "" H 5200 4400 50  0001 C CNN
	1    5200 4400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 4400 5200 4300
Wire Wire Line
	5500 1900 5500 2200
$Comp
L Device:R R8
U 1 1 60206E23
P 6100 2300
F 0 "R8" H 6170 2346 50  0000 L CNN
F 1 "10k" H 6170 2255 50  0000 L CNN
F 2 "My_Misc:R_Axial_DIN0207_L6.3mm_D2.5mm_P10.16mm_Horizontal_larger_pads" V 6030 2300 50  0001 C CNN
F 3 "~" H 6100 2300 50  0001 C CNN
	1    6100 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 3500 6100 2450
Connection ~ 6100 3500
Wire Wire Line
	6100 2150 6100 1950
$EndSCHEMATC
