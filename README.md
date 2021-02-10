# arduTester
An arduino component tester based on https://create.arduino.cc/projecthub/plouc68000/ardutester-v1-13-the-arduino-uno-transistor-tester-dbafb4

## Prerequisites

>$ sudo apt install avr-libc avrdude gcc-avr make git  


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

