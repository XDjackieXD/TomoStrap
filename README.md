 # TomoStrap
 
 The code residing in the "microcontroller" and "pebble" directories was written by me, Jakob (Jack/XDjackieXD), as part of the diploma thesis you are required to do for your A Level at a higher technical school in Austria.
 The KiCAD files inside the "hardware" directory, designed by Jakob Z., represent the hardware the aforementioned code runs on.
 
 ## Introduction
 
 The main topic of the diploma thesis was to build a device similar to http://chrisharrison.net/projects/tomo/tomo.pdf while increasing mobility by using a Pebble smartwatch as the base.
 During development I snuck in a MAX30102 pulseoximeter which basically is the main feature as of now.
 
 ## Usage
 
 ### Microcontroller Firmware
 
 To compile the Firmware for the KL03 ARM processor, it'll be the easiest route to install the kinetis design studio. The hardware abstraction layer was generated using NXP's MCUXpresso tool.
 Most parts of the code are written in a way that should allow for fairly easy porting to other microcontroller platforms.
 Currently the handlers for data received are inside the library. Changing this to callbacks should be easy though.
 The firmware isn't complete by any means but contineous SpO2 and pulse measurements are working as intended. Things that need to be addressed before "production" use include proper power management and automatic gain control for the MAX30102's LEDs and configurable ADC ranges as those values are currently hard coded which lead to unreliable measurements in many situations.
 
 ### Pebble App
 
 The included pebble app is able to display any SpO2 and pulse data sent by the microcontroller. Impedance data is currently only printed into the debug log of the Pebble app.
 
 ## License
 
 All code written by me and the SpO2 calculation library is licensed under the MIT license (LICENSE-software.txt).
 The dstring library used for SpO2 debugging using FlexiPlot is licensed under the Creative Commons license (LICENSE-dstring.txt).
 All other source code files included in the microcontroller firmware directory (everything not inside of the "source" directory) is licensed under the respective licenses stated in the source file headers.

 The hardware is Licensed under the terms of the Creative Commons Attribution Sharealike 4.0 license (LICENSE-hardware.txt) with the exception of the kicad library by Michael Cousins which is licensed under the terms of the Creative Commons Attribution Sharealike 3.0 Unported license (LICENSE-kicad-libs.txt).
