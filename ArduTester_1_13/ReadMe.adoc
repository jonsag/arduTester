
:Author: plouc68000
:Email: plouc68000@gmail.com
:Date: 14/10/2018
:Revision: ArduTester V1.13
:License: OSHW

= Project: {Project}
Porting of the files from TransistorTester V1.13 in the Arduino Editor, 
porting first to Arduino Mega, then to Arduino UNO.

The project uses widely the files posted below, same filenames, just .ino files.

https://www.mikrocontroller.net/svnbrowser/transistortester/Software/trunk/

https://github.com/svn2github/transistortester/tree/master/Software/trunk

Description and credits you find here:
https://www.mikrocontroller.net/articles/AVR_Transistortester

The Project now "Verify" and "Upload" to a Arduino/Genuino Uno with the standard Arduino tools

Differences with Arduino_uno version published as .hex

-Works with a LCD 4 bits parallel instead of serial port
-Test_button moved from DIGITAL 6 to ANALOG IN A3 to free DIGITAL 2...7 for LCD
-DIGITAL 0-1 is reserved for Rx,Tx (Monitor). Not active in this version.

== Step 1: Installation

1. Open the file
2. Edit as you like
3. Release to the World!

== Step 2: Assemble the circuit

Assemble the circuit following this instruction:

1. Look at your Arduino/Genuino Uno pins:

-Pin ANALOG IN A0 (PC0) is TP1 ( ArduinoTester Pin 1 )
-Pin ANALOG IN A1 (PC1) is TP2 ( ArduinoTester Pin 2 ) 
-Pin ANALOG IN A2 (PC2) is TP3 ( ArduinoTester Pin 3 )

-Between A0 and DIGITAL 8 (PB0)  connect a 680 Ohm  Resistor
-Between A0 and DIGITAL 9 (PB1)  connect a 470 kOhm Resistor
-Between A1 and DIGITAL 10 (PB2) connect a 680 Ohm  Resistor
-Between A1 and GIGITAL 11 (PB3) connect a 470 kOhm Resistor
-Between A2 and DIGITAL 12 (PB4) connect a 680 Ohm  Resistor
-Between A1 and DIGITAL 13 (PB5) connect a 470 kOhm Resistor

-Pin ANALOG IN A3 to a 10 kOhm pullup Resistor at 5V and via the Test Button to GND


2. Look at your 2 Line LCD HD44780 Compatible ( 4 Bits Data ):

-Connect DIGITAL 5  to LCD DB4
-Connect DIGITAL 4  to LCD DB5
-Connect DIGITAL 3  to LCD DB6
-Connect DIGITAL 2  to LCD DB7
-Connect DIGITAL 7  to LCD RS
-Connect DIGITAL 6  to LCD E

-LCD LED- to GND
-LCD LED+ to 3,3V
-LCD VDD  to 5V
-LCD VSS  to GND
-LCD R/W  to GND
-LCD VO   to 10 kOhm Pot. ( Contrast Adjust. )

== Step 3: Load the code

Select Arduino/Genuino Uno and
Upload the code contained in this sketch on to your board

=== Folder structure

....
 TT_1_13_UNO             => Arduino sketch folder
  ├── TT_13oct.ino       => main Arduino file
  ├── schematics.png     => (optional) if someone wants to do it ...
  ├── layout.png         => (optional) if someone wants to do it ...
  └── ReadMe.adoc        => this file
....

=== License
This project is released under a {OSHW} License.

=== Contributing
To contribute to this project please contact plouc68000 <plouc68000@gmail.com>

=== BOM
Add the bill of the materials you need for this project.

|===
| ID | Part name           | Part number | Quantity
|    | Arduino UNO R3      |             | 1       
|    | LCD Module 1602     |             | 1        
|    | Resistor 680 Ohm    |             | 3   
|    | Resistor 470 kOhm   |             | 3
|    | Resistor 10  kOhm   |             | 1
|    | Pot. 10 kOhm        |             | 1
|    | Test Button         |             | 1  
|===


=== Help
This document is written in the _AsciiDoc_ format, a markup language to describe documents. 
If you need help you can search the http://www.methods.co.nz/asciidoc[AsciiDoc homepage]
or consult the http://powerman.name/doc/asciidoc[AsciiDoc cheatsheet]
