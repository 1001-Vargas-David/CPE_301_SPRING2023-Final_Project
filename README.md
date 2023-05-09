# CPE_301_SPRING2023-Final_Project
## Overview <br>
&emsp;&emsp; For this project, we implemented a cooling system. The system can turn on and off by the press of a button by the user. The system idles whenever the temperature reaches the threshold of 70 degrees Fahrenheit or less (implying the room is cool enough based on the sensor). The system enters an error state whenever water levels are too low. The direction of the ventilation cannot be done in this state. The fan motor runs if the user turns on the system and the temperature is below the threshold and there is enough water for cooling. The system is disabled if the user decides to turn the system off. No monitoring of the temperature and humidity is done in this state. <br>
## Constraints <br>
&emsp;&emsp; Since the water sensor is connected to an analog port and can only take 5v, a power supply was needed, where the power supply was implemented on the board and the source came from the wall. Connecting it to a battery was not enough to get the system to work to its full capacity. Another constraint regarding the water sensor is that each sensor reads the values based on the condition it’s in, so it may vary from sensor to sensor. Since the sensor grows rust over time after being put in the water, that may also affect the reading, thus scraping off the rust is needed. Another constraint is that since we couldn’t use the serial library functions, we have to use the UART meaning we could only output one character at a time. Displaying the time required us to break the number down and convert everything to an unsigned character so we could then display it. Since we had to use bitwise operation for fan, when the fan was turned on, it was set to a default speed. While running the cooling system, the water sensor started to get a little hot the longer it ran, so it limits the use of the water sensor which is essential to the system. <br>

## Functions <br>
Function name: setup <br>
Function return type: void <br>
Function parameters: none <br>
Functionality:	sets up our program prior to all the action occurring in the loop function which executes infinitely many times. This function initializes the serial port for the USART0, sets the pins for LEDs, buttons for the stepper motor, and buttons for the system on/off and reset button to output, initialization for the DHT sensor communication, initialization for LCD communication and display grid, along with initializing the ADC. <br> <br>

Function name: loop <br>
Function return type: void <br>
Function parameters: none <br>
Functionality: displays the current time and date every minute, changes state (if needed) to one of the following: disabled, idle, error, or running. It also checks to see if the button was pressed to determine if we should turn on or off the system. <br> <br>

Function name: ISR <br>
Function return type: none <br>
Function parameters: TIMER1_OVF_vect <br>
Functionality: interrupt service routine function which handles timer overflow. It stops the timer and then sets the timer to 0. <br> <br>


Function name: adc_init() <br>
Function return type: void <br>
Function parameters: none <br>
Functionality: initializes the Analog to Digital Converter (ADC). <br> <br>

Function name: adc_read <br>
Function return type: unsigned int <br>
Function parameters: unsigned char <br>
Functionality: reads analog input using an Analog to Digital Converter (ADC). <br> <br>

Function name: DISABLED <br>
Function return type: int <br>
Function parameters: none <br>
Functionality: the function turns on the yellow LED to signal we are in the disabled state while turning the other LEDs off. The function displays that the motor is off if it was on along with the time of the occurrence. The user has access to rotate the stepper motor. The LCD is cleared. <br> <br>

Function name: IDLE <br>
Function return type: int <br>
Function parameters: none <br>
Functionality: turns on the green LED to signal we are in the idle state while turning the other LEDs off. The function stops the fan motor; however the stepper motor can still rotate direction if the user desires to do so. The water level and temperature determine the state the user will enter next. <br> <br>

Function name: ERROR <br>
Function return type: int <br>
Function parameters: none <br>
Functionality: turns on the red LED to signal we are in the error state while turning the other LEDs off. The function also displays a message that the motor is off along with the time of the occurrence. The function turns off the fan and the stepper motor has no access to change ventilation direction. If the water level is too low, the LCD will display that the water level is low. The user can turn the system off. <br> <br>

Function name: Running <br>
Function return type: int <br>
Function parameters: none <br>
Functionality: turns on the blue LED to signal we are in the running state while turning the other LEDs off. The function also turns on the fan, displays the motor is on along with the time of the occurrence, functionality of the stepper motor so the user can rotate it either clockwise or counterclockwise if wanted, displays the info to the LCD, and checks many other factors such as water level and temperature to determine it return value to determine which state to enter. <br>


Function name: LCDDisplayInfo <br>
Function return type: void <br>
Function parameters: int <br>
Functionality: takes in a value (specifically the water level) and if the value meets the criteria, it will enter the appropriate if statement to execute the following: display the temperature and humidity read by the sensor from the room along with its associated water level based on the value from the parameter (medium or high). <br> <br>

Function name: timeDisplay <br>
Function return type: void <br>
Function parameters: none <br>
Functionality: reads the current time from the system and then calls the printTimeInfo function if the read was successful. <br> <br>

Function name: printTimeInfo <br>
Function return type: void <br>
Function parameters: None <br>
Functionality: displays the time that was last read from the system. <br> <br>

Function name: print2digits <br>
Function return type: void <br>
Function parameters: int <br>
Functionality: breaks apart the 2-digit number that was passed in by the parameter, converts each digit to an unsigned character to display the information correctly to the user. <br> <br>

Function name: print4digits <br>
Function return type: void <br>
Function parameters: int <br>
Functionality: breaks apart the 4-digit number that was passed in from the parameter, converts each digit to an unsigned character to display the information correctly to the user. <br> <br>

Function name: printStepperRotation <br>
Function return type: void <br>
Function parameters: bool <br>
Functionality: displays which way the stepper motor is rotating (clockwise or counterclockwise) based on the parameter. <br> <br>

Function name: U0init <br>
Function return type: void <br>
Function parameters: unsigned long <br>
Functionality: Takes in the BAUD rate that wants to be used to initialize the USART0 which allows for serial data communication. <br> <br>

Function name: U0kbhit <br>
Function return type: unsigned char <br>
Function parameters: None <br>
Functionality: Read USART0 RDA status bit and return non-zero true if set. <br> <br>


Function name: U0pdata <br>
Function return type: void <br>
Function parameters: None <br>
Functionality: Waits for USART0 TBE to be set then and then it writes the character received in the parameter to transmit buffer which then is eventually displayed to the screen. <br>


## Tools
**Software** <br>
Arduino IDE <br>
**Physical components** <br>
&emsp;&emsp; Male-to-female wires, Male-to-male wires (jumper wires), Arduino Mega 2560 Controller Board, Breadboards, 4 LEDs (one of each: red, yellow, green, blue), button (small), resistors (1k), USB cable, Power supply module, ULN2003 Stepper Motor Driver Board, LCD 1602 Module, Stepper Motor, Water Lever Detection Sensor Module, L293D, Fan Blade and 3V DC Motor (with wire), DS1307 RTC Module, DHT11 Temperature and Humidity Module, 9V 1A adapter.
## Resources/References
How do I use a Real Time Clock with Arduino? RTC 1307 (https://www.youtube.com/watch?v=KgqMOPErWjk) <br>
How to Set Up the DHT11 Humidity Sensor on an Arduino (circuitbasics.com) (https://www.circuitbasics.com/how-to-set-up-the-dht11-humidity-sensor-on-an-arduino/) <br>
Interrupts: Button Input: On/off state change (energiazero.org) (https://www.circuitbasics.com/how-to-set-up-the-dht11-humidity-sensor-on-an-arduino/) <br>
Fan and dc motor: Arduino Tutorial 37: Understanding How to Control DC Motors in Projects (https://www.youtube.com/watch?v=fPLEncYrl4Q) <br>
Pin layout for Arduino Mega 2560: https://www.electronicshub.org/wp-content/uploads/2021/01/Arduino-Mega-Pinout.jpg <br>
Data sheet: http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2549-8-bit-AVR-Microcontroller-ATmega640-1280-1281-2560-2561_datasheet.pdf
