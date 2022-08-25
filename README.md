# iontra-led
iontra

The zip file include:
- a header file led_interface.h
- main.c which includes definitions of core function for the led_interface as well as an implementation of it for a stm32f401 board.
-set_brightness, set_led, and led_init are independent of the hardware.
- led_show, gpio_init, Pwm_init are board specific function. So for this code to work on another board those functions need to be modified or support for another board added.

all the source and header files are located inside the folder named core
