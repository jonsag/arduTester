# arduTester

An arduino component tester based on [this](https://create.arduino.cc/projecthub/plouc68000/ardutester-v1-13-the-arduino-uno-transistor-tester-dbafb4)

## Prerequisites

>$ sudo apt install make git  
>$ sudo apt install avr-libc avrdude gcc-avr subversion  

## Download sources

>$ svn checkout svn://mikrocontroller.net/transistortester  

## Hardware

### ATmega320

Pin no ->
1 -> Reset
2 -> D0
3 -> D1
4 -> D2 -> LCD pin 14
5 -> D3 -> LCD pin 13
6 -> D4 -> LCD pin 12
7 -> Vcc -> Vcc
8 -> GND -> GND
9 -> Crystal
10 -> Crystal
11 -> D5 -> LCD pin 11
12 -> D6 -> LCD pin 6
13 -> D7 -> LCD pin 4
14 -> D8 -> TP1 680R
15 -> D9 -> TP1 470k
16 -> D10 -> TP2 680R
17 -> D11 -> TP2 470k
18 -> D12 -> TP3 680R
19 -> D13 -> TP3 470k
20 -> Vcc -> Vcc
21 -> Aref
22 -> GND -> GND
23 -> A0 -> TP1
24 -> A1 -> TP2
25 -> A2 -> TP3
26 -> A3 -> Test button
27 -> A4
28 -> A5


## My notes

C:\Users\alex\AppData\Local\Arduino15\packages\arduino\tools\avrdude\6.3.0-arduino9/bin/avrdude -CC:\Users\alex\AppData\Local\Arduino15\packages\arduino\tools\avrdude\6.3.0-arduino9/etc/avrdude.conf -c usbasp -B 20 -p m328p -P usb -U flash:w:./TransistorTester.hex:a -U eeprom:w:./TransistorTester.eep:a

Steps:
1. Install some packages on Ubuntu:
apt-get install avr-libc avrdude gcc-avr make git
2. Clone the git repository
git clone https://github.com/svn2github/transistortester
3. Copy the makefile from the folder of this lcd to the default (it may not necessary and can be built in it's original location)
cp transistortester/Software/trunk/ST7735/Makefile transistortester/Software/trunk/default/
4. Go to the default directory
cd transistortester/Software/trunk/default
5. Edit the Makefile and change the programmer setting
PROGRAMMER=usbtiny
6. Build and upload the code
make
make upload

