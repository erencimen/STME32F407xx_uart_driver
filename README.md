# STME32F407xx_uart_driver

 MCU Specific header file(stm32F407xx.h), GPIO driver header file(stm32f407xx_gpio_driver.h) and UART driver header file(STM32F407xx_uart_driver.h) are prepared for only this project.


This drivers supports only asynchronous (UART) communication.

For test the UART driver STM32F4DISCOVERY Board and ARDUNIO UNO R3 boards are used.

#TEST SCENARIO
When the user wake-up button on STM32F4DISCOVERY Board is pressed data will be sent to ARDUNIO UNO R3 Board and ARDUNIO Board changes the case of alphabets (lower case to upper case and vice versa) then sends data back to STM32F4DISCOVERY Board.

#CONNECTIONS

                                    STM32F4DISCOVERY                  ARDUNIO UNO

                                         PA2  ------------------------>  1
                                         PA3  ------------------------>  0

                                         GND   ----------------------->  GND
