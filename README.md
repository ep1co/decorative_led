# decorative_led
Controlling leds with nucleo F401RE and receive command from terminal emulator.

This code controls 3 leds with a single push button and turn on each led by inserting command to the terminal emulator. There are plenty of ideas to customise the 
project like adding more leds/buttons, using external I2C monitor (OLED/LCD) instead of emulators like PuTTY, terraterm,...

Before running the code, make sure to install a compatible IDE first (STM32CubeIDEor Keil MDK-ARM is an ideal choice), then create an STM32 project using C language, 
choose the right board. 

When the code ran successful, open the terminal emulator and set connection type to serial, choose your right COM, set the speed, or baud rate
to 115200 then click run/open. Now you can try to insert the command, push the button and see the result.


