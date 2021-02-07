// ardAVRProgrammer_enclosure.scad

// Design: Jon Sagebrand
// jonsagebrand@gmail.com

// pin headers by Niccolo Rigacci <niccolo@rigacci.org>, https://github.com/RigacciOrg/openscad-rpi-library

// arduino module by ?
include <../../myCAD/OpenSCADlibraries/Arduino.scad>

// LED module by ?
include <../../myCAD/OpenSCADlibraries/led.scad>

// NOP scad library by nophead, https://github.com/nophead/NopSCADlib
include <../../myCAD/OpenSCADlibraries/NopSCADlib/vitamins/dip.scad>
include <../../myCAD/OpenSCADlibraries/NopSCADlib/vitamins/displays.scad>
include <../../myCAD/OpenSCADlibraries/NopSCADlib/vitamins/buttons.scad>
include <../../myCAD/OpenSCADlibraries/NopSCADlib/vitamins/axials.scad>

// lay out for printing
print = true;

showBottom = true;
showLid = false;

// show arduino board and shield
showArd = true;
showShield = true;

// shield type
// 0: no shield
// 1: ardAVRProgrammer
// 2: arduTester
shieldType = 2;

// make the hole for the arduino USB and power input
usbHole = true;
powHole = false;

// arduino stand offs
soBotHeight = 4;
soBotDia = 9;

soTopHeight = 1;
soTopDia = 6.5;

ardMountHoleDia = 4.5;

ardPosts = [[14, 2.5, soBotHeight / 2], 
            [66.1, 7.6, soBotHeight / 2], 
            [66.1, 35.5, soBotHeight / 2], 
            [15.3, 50.7, soBotHeight / 2]];

// arduino uno dimensions
ardWidth = 68.6;
ardDepth = 53.3;
ardHeight = 10.9 + 1.6;

// casing
casWallThick = 1.5;

extraWidthL = shieldType == 0 ? 2 : (shieldType == 1 ? 1 : (shieldType == 2 ? 12 : 5 ) );
extraWidthR = shieldType == 0 ? 2 : (shieldType == 1 ? 18.5 : (shieldType == 2 ? 35 : 5 ) );
extraDepthD = shieldType == 0 ? 8.5 : (shieldType == 1 ? 6.5 : (shieldType == 2 ? 11.5 : 5 ) );
extraDepthU = shieldType == 0 ? 8.5 : (shieldType == 1 ? 12 : (shieldType == 2 ? 12.5 : 5 ) );

extraHeight = shieldType == 0 ? 2 : (shieldType == 1 ? 5 : (shieldType == 2 ? 18 : 5 ) );

// casing lid
lidInset = 2;
lidOffset = 0;

lidScrewHoleDia = 4.5;
lidScrewPostDia = 9;
lidScrewHoleDepth = 7;

lidThick = 1.5;

lidHoleDia = 3.5;

lidRidgeWidth = 2;

tolerance = 0.2;

///// no editing below this line
roundness = 100;

// calculations of casing outer boundaries                                     
casXL = -casWallThick - extraWidthL;
casXR = ardWidth + extraWidthL + extraWidthR + casWallThick * 2 + casXL;
casYD = -casWallThick - extraDepthD;
casYU = ardDepth + extraDepthD + extraDepthU + casWallThick * 2 + casYD;

// lid screw posts                                                             
postsY = soBotHeight + soTopHeight + ardHeight + extraHeight - lidScrewHoleDepth / 2 - lidInset;

lidPostsPos = [[casXL + lidScrewPostDia / 2 + lidOffset, 
                casYD + lidScrewPostDia / 2 + lidOffset, 
                postsY],
	       [casXR - lidScrewPostDia / 2 - lidOffset, 
                casYD + lidScrewPostDia / 2 + lidOffset, 
                postsY],
	       [casXR - lidScrewPostDia / 2 - lidOffset, 
                casYU - lidScrewPostDia / 2 - lidOffset, 
                postsY],
	       [casXL + lidScrewPostDia / 2 + lidOffset, 
                casYU - lidScrewPostDia / 2 - lidOffset, 
                postsY]];

dropPos = [[casXL + casWallThick, 
            casYD + casWallThick, 
            postsY - lidScrewHoleDepth * 1.5],
	   [casXR - casWallThick, 
            casYD + casWallThick, 
            postsY - lidScrewHoleDepth * 1.5],
	   [casXR - casWallThick, 
            casYU - casWallThick, 
            postsY - lidScrewHoleDepth * 1.5],
	   [casXL + casWallThick, 
            casYU - casWallThick, 
            postsY - lidScrewHoleDepth * 1.5]];

////////// the drawing //////////

if ((showArd) && (!print)) {
  translate([0, 53.3, soBotHeight + soTopHeight])
    rotate([0, 0, -90])
    arduinoUno();
 }

if ((showShield) && (!print) && (shieldType)) {
  if (shieldType == 1) {
    translate([0.45, 0.35, 17.9])
      shieldType1();
  } else if (shieldType == 2) {
    translate([-10.9, -3.45, 17.9])
      shieldType2();
  }
}

union() {
  if (showBottom) {
    ardMountPosts();
    bottomCasing();
  }
}

//ardMountPosts();
if (showLid) {
  if (print) {
    rotate([180, 0, 0])
      translate([0, 30, 0])
      difference() {
        lid();
        if (shieldType == 1) {
          lidOpeningsType1();
        }
      }
  } else {
    translate([0, 0, soBotHeight + soTopHeight + ardHeight + extraHeight])
    difference() {
      lid();
      if (shieldType == 1) {
        lidOpeningsType1();
      }
    }
  }
 }

module ardMountPosts() {
  for (i = [0 : 1 : 3]) { 
    
    translate(ardPosts[i])
      difference() {
      union() {
	color("blue")
	  cylinder(h = soBotHeight, r1 = soBotDia / 2, r2 = soTopDia / 2, 
		   center = true, $fn = roundness);
	
	color("green")
	  translate([0, 0, soBotHeight / 2 + soTopHeight / 2])
	  cylinder(h = soTopHeight, r1 = soTopDia / 2, r2 = soTopDia / 2, 
		   center = true, $fn = roundness);
	
      }
      
      color("red")
	translate([0, 0, soBotHeight / 2])
        cylinder(h = soBotHeight + soTopHeight, 
		 r1 = ardMountHoleDia / 2, r2 = ardMountHoleDia / 2, 
		 center = true, $fn = roundness);
      
    }
  }
}

module bottomCasing() {
  
  union() {
    translate([-extraWidthL - casWallThick, -extraDepthD - casWallThick, -casWallThick])
      difference() { // bottom part
      color("lightgrey")
	cube([ardWidth + extraWidthL + extraWidthR + casWallThick * 2, 
	      ardDepth + extraDepthD + extraDepthU + casWallThick * 2, 
	      soBotHeight + soTopHeight + ardHeight + casWallThick + extraHeight]);
      
      translate([casWallThick, casWallThick, casWallThick + 0.01])
	color("red")
	cube([ardWidth + extraWidthL + extraWidthR, 
	      ardDepth + extraDepthD + extraDepthU, 
	      soBotHeight + soTopHeight + ardHeight + extraHeight]);
      
      // usb port
      if (usbHole) {
      translate([-casWallThick / 2, 30.2 + casWallThick + extraDepthD, 
		 casWallThick + soBotHeight + soTopHeight])
	color("red")
	cube([casWallThick * 2, 12 + 2, 10.9 + 2]);
      }

      // power supply
      if (powHole) {
	translate([-casWallThick / 2, 3.3 + casWallThick + extraDepthD, 
		   casWallThick + soBotHeight + soTopHeight + 2])
	  union() {
	  //color("red")
	  //cube([casWallThick, 8.9 + 2, 8.9 / 2 + 1]);
	  
	  translate([0, 8.9 / 2 + 1, 8.9 / 2 + 1])
	    rotate([0, 90, 0])
	    cylinder(h = casWallThick * 2, r1 = 8.9 / 2 + 1, r2 = 8.9 / 2 + 1, $fn = roundness);
	}
      }
    }
    
    // lid screw posts
    translate([0, 0, 0])
      for (j = [0 : 1 : 3]) {
	difference() {
	  hull() {
	    translate(lidPostsPos[j])
	      cylinder(h = lidScrewHoleDepth, 
		       r1 = lidScrewPostDia / 2, r2 = lidScrewPostDia / 2, 
		       center = true, $fn = roundness);
	    
	    translate(dropPos[j])
	      cylinder(h = 0.1, r1 = 0.1, r2 = 0.1, center = true, $fn = roundness);
	    
	  }
	  
	  // screw Holes
	  translate(lidPostsPos[j])
	    color("red")
	    cylinder(h = lidScrewHoleDepth, 
		     r1 = lidScrewHoleDia / 2, r2 = lidScrewHoleDia / 2, 
		     center = true, $fn = roundness);
	}
      }    
  }
}

//------------------------------------------------------------------------
// OpenSCAD models of miscellaneous components and devices:
// Pin headers, SD-Card, Edimax WiFi nano dongle, etc.
//
// Author:      Niccolo Rigacci <niccolo@rigacci.org>
// Version:     1.0 2017-12-14
// License:     GNU General Public License v3.0
//------------------------------------------------------------------------

//------------------------------------------------------------------------
// Matrix of 2.54 mm pins.
//------------------------------------------------------------------------
module pin_headers(cols, rows) {
  w = 2.54; h = 2.54; p = 0.65;
  for(x = [0 : (cols -1)]) {
    for(y = [0 : (rows  - 1)]) {
      translate([w * x, w * y, 0]) {
	union() {
	  color("black") 
	    cube([w, w, h]);
	  color("gold")  
	    translate([(w - p) / 2, (w - p) / 2, -3]) cube([p, p, 11.54]);
	}
      }
    }
  }
}

//------------------------------------------------------------------------
// Matrix of 2.54 mm female connectors.
//------------------------------------------------------------------------
module pin_female(cols, rows=1) {
    w = 2.54; h = 8.5; p = 0.65;
    for(x = [0 : (cols -1)]) {
        for(y = [0 : (rows  - 1)]) {
            translate([w * x, w * y, 0]) {
                union() {
                    color([0.2, 0.2, 0.2]) difference() {
                        cube([w, w, h]);
                        translate([(w - p) / 2,(w - p) / 2,h - 6]) cube([p, p, 6.1]);
                    }
                    color("gold")  translate([(w - p) / 2, (w - p) / 2, -3]) cube([p, p, 3]);
                }
            }
        }
    }
}
module shieldType1() {
  translate([0, 0, 0]) // PCB
    color("green")
    cube([86.36, 58.42, 1.6]);
  
  // pin headers connecting to Arduino
  translate([28.57 - 2.54, 3.17 - 2.54, 0])
    rotate([180, 0, 90])
    pin_headers(1, 8);
  
  translate([51.44 - 2.54, 3.17 - 2.54, 0])
    rotate([180, 0, 90])
    pin_headers(1, 6);
  
  translate([19.68 - 2.54, 51.44 - 2.54, 0])
    rotate([180, 0, 90])
    pin_headers(1, 10);
  
  translate([46.35 - 2.54, 51.44 - 2.54, 0])
    rotate([180, 0, 90])
    pin_headers(1, 8);
  
  // LEDs
  //translate([79.38, 54.61, 1.6 + 1.5])
  translate([80.64, 54.61, 1.6 + 1.5])
    led(5, "blue", legs = false);

  translate([80.64, 46.35, 1.6 + 1.5])
    led(5, "red", legs = false);
  
  translate([80.64, 38.10, 1.6 + 1.5])
    led(5, "green", legs = false);
  
  // headers
  translate([59.69 - 2.54 / 2, 13.34 - 2.54 / 2, 2.54 / 2 + 0.4])
    rotate([0, 0, 0])
    pin_headers(2, 3);
  
  translate([59.69 - 2.54 / 2, 28.57 - 2.54 / 2, 2.54 / 2 + 0.4])
    rotate([0, 0, 0])
    pin_headers(2, 5);
  
  
  //  28 pin ZIF
  translate([11.43, 12.06, 1.7])
    zif28();
  /*
    translate([11.43 + 2.54 * 6.5, 12.7 + 2.54 * 2, 1.7])
    rotate([0, 0, 90])
    pdip(28, 2.54, socketed = true, w = inch(0.4));
  */
  // 20 pin ZIF
  translate([21.59, 35.56, 1.7])
    zif20();
  /*
    translate([21.59 + 2.54 * 4.5, 36.2 + 2.54 * 1.5, 1.7])
    rotate([0, 0, 90])
    pdip(20, 2.54, socketed = true, w = inch(0.3));
  */
  // 8 pin DIL
  translate([74.3 + 2.54 * 1.5, 5.08 + 2.54 * 1.5, 1.7])
    rotate([0, 0, 0])
    pdip(8, 2.54, socketed = true, w = inch(0.3));

  // capacitor
  translate([20.95, 4.17, 1.7 + 11.5 / 2])
    rotate([0, 0, 0])
    color("black")
    cylinder(h = 11.5, r1 = 2.5, r2 = 2.5, center = true, $fn = roundness);
}

module zif28() {
  
  radius = 3;
  
  translate([2.54 * 6.5 - 2, 2.54 * 2, 11.9 / 2])
    union() {
    color("lightgreen")
      difference() {
      union() {
	translate([-50.5 / 2 + radius, -15 / 2 + radius, 0])
	  cylinder(h = 11.9, r1 = radius, r2 = radius, center = true, $fn = roundness);
	
	translate([50.5 / 2 - radius, -15 / 2 + radius, 0])
	  cylinder(h = 11.9, r1 = radius, r2 = radius, center = true, $fn = roundness);
	
	translate([50.5 / 2 - radius, 15 / 2 - radius, 0])
	  cylinder(h = 11.9, r1 = radius, r2 = radius, center = true, $fn = roundness);
	
	translate([-50.5 / 2 + radius, 15 / 2 - radius, 0])
	  cylinder(h = 11.9, r1 = radius, r2 = radius, center = true, $fn = roundness);
	
	cube([50.5 - radius * 2, 15, 11.9], center = true);
	
	cube([50.5, 15 - radius * 2, 11.9], center = true);
      }
      
      // room for lever
      translate([-50.5 / 2 + 5 / 2, -15 / 2 + 3 / 2, 11.9 / 2 - 5 / 2])
	color("red")
	cube([5, 3, 5], center = true);
    }
    
    // lever
    translate([-50.5 / 2, -15 / 2 + 3 / 2, 11.9 / 2 - 5 + 1])
      rotate([0, -90, 0])
      union() {
      color("lightgrey")
	cylinder(h = 10, r1 = 1, r2 = 1, center = true, $fn = roundness);
      
      translate([0, 0, 10 / 2 + 5 / 2])
	color("white")
	cylinder(h = 5, r1 = 2, r2 = 2, center = true, $fn = roundness);
    }
    
    translate([2, 0, -11.9 / 2]) // legs under ZIF
      rotate([0, 0, 90])
      pdip(28, 2.54, socketed = false, w = inch(0.4));
    
    translate([2, 0, 11.9 / 2]) // DIP on ZIF
      rotate([0, 0, 90])
      pdip(28, 2.54, socketed = false, w = inch(0.3));
    
  }
}

module zif20() {
  
  radius = 3;
  
  translate([2.54 * 4.5 - 2, 2.54 * 1.5, 11.9 / 2])
    union() {
    color("lightgreen")
      difference() {
      union() {
	translate([-40.2 / 2 + radius, -15 / 2 + radius, 0])
	  cylinder(h = 11.9, r1 = radius, r2 = radius, center = true, $fn = roundness);
	
	translate([40.2 / 2 - radius, -15 / 2 + radius, 0])
	  cylinder(h = 11.9, r1 = radius, r2 = radius, center = true, $fn = roundness);
	
	translate([40.2 / 2 - radius, 15 / 2 - radius, 0])
	  cylinder(h = 11.9, r1 = radius, r2 = radius, center = true, $fn = roundness);
	
	translate([-40.2 / 2 + radius, 15 / 2 - radius, 0])
	  cylinder(h = 11.9, r1 = radius, r2 = radius, center = true, $fn = roundness);
	
	cube([40.2 - radius * 2, 15, 11.9], center = true);
	
	cube([40.2, 15 - radius * 2, 11.9], center = true);
      }
      
      // room for lever
      translate([-40.2 / 2 + 5 / 2, -15 / 2 + 3 / 2, 11.9 / 2 - 5 / 2])
	color("red")
	cube([5, 3, 5], center = true);
    }
    
    // lever
    translate([-40.2 / 2, -15 / 2 + 3 / 2, 11.9 / 2 - 5 + 1])
      rotate([0, -90, 0])
      union() {
      color("lightgrey")
	cylinder(h = 10, r1 = 1, r2 = 1, center = true, $fn = roundness);
      
      translate([0, 0, 10 / 2 + 5 / 2])
	color("white")
	cylinder(h = 5, r1 = 2, r2 = 2, center = true, $fn = roundness);
    }
    
    translate([2, 0, -11.9 / 2])
      rotate([0, 0, 90])
      pdip(20, 2.54, socketed = false, w = inch(0.3));
    
    translate([2, 0, 11.9 / 2])
      rotate([0, 0, 90])
      pdip(20, 2.54, socketed = false, w = inch(0.3));
    
  }
}

module lid() {
  translate([-extraWidthL - casWallThick, -extraDepthD - casWallThick, 0])
    difference() {
    union() {
      color("grey") // upper side
	cube([ardWidth + extraWidthL + extraWidthR + casWallThick * 2, 
	      ardDepth + extraDepthD + extraDepthU + casWallThick * 2, 
	      lidThick]);
      
      difference() { // ridges under lid
	translate([casWallThick + tolerance, casWallThick + tolerance, -lidInset + tolerance])
	  color("green")
	  cube([ardWidth + extraWidthL + extraWidthR + casWallThick * 2 -casWallThick * 2 - tolerance * 2, 
		ardDepth + extraDepthD + extraDepthU + casWallThick * 2 -casWallThick * 2 - tolerance * 2, 
		lidInset - tolerance]);
	
	translate([casWallThick + tolerance + lidRidgeWidth, 
		   casWallThick + tolerance + lidRidgeWidth, 
		   -lidInset + tolerance])
	  color("red")
	  cube([ardWidth + extraWidthL + extraWidthR + casWallThick * 2 -casWallThick * 2 - lidRidgeWidth* 2 - tolerance * 2, 
		ardDepth + extraDepthD + extraDepthU + casWallThick * 2 -casWallThick * 2 - lidRidgeWidth * 2 - tolerance * 2, 
		lidInset - tolerance]);
      }
    }
    
    // screw holes
    for (i = [0 : 1 : 3]) {
      translate([extraWidthL + casWallThick, 
		 extraDepthD + casWallThick, 
		 -postsY - lidThick / 2 + lidInset / 2 - tolerance * 2])
	translate(lidPostsPos[i])
  color("red")
	cylinder(h = lidThick + lidInset, 
		 r1 = lidHoleDia / 2, r2 = lidHoleDia / 2, 
		 center = true, $fn = roundness);
    }
  }
}

module lidOpeningsType1() {
  xOffset = casWallThick + extraWidthL; // + 0.4;
  yOffset = casWallThick + extraDepthD; // + 0.3;
  
  translate([-extraWidthL - casWallThick, -extraDepthD - casWallThick, 0])
  union() {
  // zif 28
  translate([xOffset + 26.04 + 0.6, 
  yOffset + 16.18, 
  (lidThick + lidInset) / 2 - lidInset + tolerance])
    color("red")
    cube([50.5 + 2, 15 + 2, lidThick + lidInset], center = true);
  
  // zif 20
  translate([xOffset + 30.795 + 0.4, 
  yOffset + 39.37 + 0.3, 
  (lidThick + lidInset) / 2 - lidInset + tolerance])
    color("red")
    cube([40.2 + 2, 15 + 2, lidThick + lidInset], center = true);
  
  // 8-pin DIL
  translate([xOffset + 78.11 - 0.6, 
  yOffset + 8.89 + 0.4, 
  (lidThick + lidInset) / 2 - lidInset + tolerance])
    color("red")
    cube([10.16 + 2, 10.16 + 2, lidThick + lidInset], center = true);
  
  // 6 pin header
  translate([xOffset + 60.96 - 0.6, 
  yOffset + 15.88 - 0., 
  (lidThick + lidInset) / 2 - lidInset + tolerance])
    color("red")
    cube([2.54 * 2 + 2, 2.54 * 3 + 2, lidThick + lidInset], center = true);
  
  // 10 pin header
  translate([xOffset + 60.96 - 0.6, 
  yOffset + 33.66 - 0.2, 
  (lidThick + lidInset) / 2 - lidInset + tolerance])
    color("red")
    cube([2.54 * 2 + 2, 2.54 * 5 + 2, lidThick + lidInset], center = true);
  
  // blue LED
  translate([xOffset + 80.64 - 1.5, 
  yOffset + 54.61 + 0.3, 
  (lidThick + lidInset) / 2 - lidInset + tolerance])
    color("red")
    cylinder(h = lidThick + lidInset, r1 = 3, r2 = 3, center = true, $fn = roundness);
  
  // red LED
  translate([xOffset + 80.64 - 1.5, 
  yOffset + 46.35 + 0.3, 
  (lidThick + lidInset) / 2 - lidInset + tolerance])
    color("red")
    cylinder(h = lidThick + lidInset, r1 = 3, r2 = 3, center = true, $fn = roundness);
  
  // green LED
  translate([xOffset + 80.64 - 1.5, 
  yOffset + 38.1 + 0.3, 
  (lidThick + lidInset) / 2 - lidInset + tolerance])
    color("red")
    cylinder(h = lidThick + lidInset, r1 = 3, r2 = 3, center = true, $fn = roundness);

  // capacitor
  translate([xOffset + 20.95 + 0.4, 
  yOffset + 4.17 + 0.3, 
  (lidThick + lidInset) / 2 - lidInset + tolerance])
    color("red")
    cylinder(h = lidThick + lidInset, r1 = 3, r2 = 3, center = true, $fn = roundness);
  }
}

module shieldType2() {
  translate([0, 0, 0]) // PCB
    color("green")
    cube([113.67, 60.96, 1.6]);
  
  // pin headers connecting to Arduino
  translate([40.01 - 2.54, 6.99 - 2.54, 0])
    rotate([180, 0, 90])
    pin_headers(1, 8);
  
  translate([62.87 - 2.54, 6.99 - 2.54, 0])
    rotate([180, 0, 90])
    pin_headers(1, 6);
  
  translate([31.11 - 2.54, 55.24 - 2.54, 0])
    rotate([180, 0, 90])
    pin_headers(1, 10);
  
  translate([57.78 - 2.54, 55.24 - 2.54, 0])
    rotate([180, 0, 90])
    pin_headers(1, 8);

  // pin headers for LCD
  translate([40.01 - 2.54 / 2,  38.1 + 2.54 / 2, 1.6])
    rotate([0, 0, 270])
    pin_female(1, 16);

  // LCD display
  translate([40.01 + 32, 38.1 - 15.5, 21.5])
    rotate([0, 180, 0])
    display(LCD1602A);

  // test button
  translate([100.33, 50.16, 1.7])
    rotate([0, 0, 0])
    square_button(button_12mm, "yellow");

  // test header J1
  translate([6.35 - 2.54 / 2, 10.16 - 2.54 / 2, 1.6])
    rotate([0, 0, 0])
    pin_female(1, 3);

  // test header J3
  translate([15.24 - 2.54 / 2, 10.16 - 2.54 / 2, 1.6])
    rotate([0, 0, 0])
    pin_female(1, 3);

  // resistor R7
  translate([11.43, 54.61, 2])
    rotate([0, 0, 0])
    ax_res(res1_4, 470000);

  // resistor R6
  translate([11.43, 48.26, 2])
    rotate([0, 0, 0])
    ax_res(res1_4, 680);

  // resistor R5
  translate([11.43, 42.55, 2])
    rotate([0, 0, 0])
    ax_res(res1_4, 470000);

  // resistor R4
  translate([11.43, 36.2, 2])
    rotate([0, 0, 0])
    ax_res(res1_4, 680);

  // resistor R3
  translate([11.43, 30.48, 2])
    rotate([0, 0, 0])
    ax_res(res1_4, 470000);

  // resistor R2
  translate([11.43, 24.13, 2])
    rotate([0, 0, 0])
    ax_res(res1_4, 680);

  // resistor R1
  translate([61.59, 28.57, 2])
    rotate([0, 0, 0])
    ax_res(res1_4, 10000);

  // power input header
  translate([ 85.09 + 2.54 * 1.5, 45.09 - 2.54 / 2, 1.6])
    rotate([0, 0, 90])
    pin_headers(1, 2);
}

module lidOpeningsType2() {
  xOffset = casWallThick + extraWidthL; // + 0.4;
  yOffset = casWallThick + extraDepthD; // + 0.3;
}