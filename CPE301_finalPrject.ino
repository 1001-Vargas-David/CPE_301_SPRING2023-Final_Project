#include <LiquidCrystal.h>
#include <DHT.h>
#include <Stepper.h>
#include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>

//UART 
#define RDA 0x80
#define TBE 0x20

volatile unsigned char *myUCSR0A = (unsigned char)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char)0x00C6;

//speedPin
volatile unsigned char* port_d = (unsigned char*) 0x2B; 
volatile unsigned char* ddr_d  = (unsigned char*) 0x2A;
volatile unsigned char* pin_d  = (unsigned char*) 0x29; 

//direction1
volatile unsigned char* port_g = (unsigned char*) 0x34; 
volatile unsigned char* ddr_g  = (unsigned char*) 0x33;
volatile unsigned char* pin_g  = (unsigned char*) 0x32; 

//direction2
volatile unsigned char* port_e = (unsigned char*) 0x2E; 
volatile unsigned char* ddr_e  = (unsigned char*) 0x2D;
volatile unsigned char* pin_e  = (unsigned char*) 0x2C; 

//Green LED
//volatile unsigned char* port_g = (unsigned char*) 0x34; 
//volatile unsigned char* ddr_g  = (unsigned char*) 0x33; 

//Yellow LED
//volatile unsigned char* port_d = (unsigned char*) 0x2B; 
//volatile unsigned char* ddr_d  = (unsigned char*) 0x2A; 

//Red LED
volatile unsigned char* port_l = (unsigned char*) 0x10B; 
volatile unsigned char* ddr_l  = (unsigned char*) 0x10A;
volatile unsigned char* pin_l  = (unsigned char*) 0x109; 

//Activate Button
volatile unsigned char* port_b = (unsigned char*) 0x25; 
volatile unsigned char* ddr_b  = (unsigned char*) 0x24;
volatile unsigned char* pin_b  = (unsigned char*) 0x23; 

//analogWrite pin 22 for constrast of LCD
volatile unsigned char* port_a = (unsigned char*) 0x22; 
volatile unsigned char* ddr_a  = (unsigned char*) 0x21;
volatile unsigned char* pin_a  = (unsigned char*) 0x20; 

//timers
volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;

//define PORT H registers pointers
volatile unsigned char* port_h = (unsigned char*) 0x102; 
volatile unsigned char* ddr_h = (unsigned char*) 0x101; 
volatile unsigned char* pin_h = (unsigned char*) 0x100; 

//adc pointers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//stepper motor variables
int stepsPerRevolution = 2048;
int stepperMSpeed = 10;

Stepper myStepper(stepsPerRevolution, 8, 10, 9, 11);

//timer variables
tmElements_t tm;
int newMinute = -1;
int newSecond = -1;
bool readTime = 0;

//pin A4
#define SIGNAL_PIN 4

#define DHT11_PIN 6
int value = 0; // variable to store the sensor value
volatile int state = 0;
bool timerRunning = 0;
volatile bool lowFlag = 0;
volatile bool medFlag = 0;
volatile bool highFlag = 0;
volatile bool ONFlag = 0;
volatile bool OFFFlag = 0;
volatile bool leftFlag = 0;
volatile bool rightFlag = 0;

volatile bool errorFlag = 0;

DHT HT(DHT11_PIN, DHT11);
LiquidCrystal lcd(24, 26, 28, 30, 32, 34);

void setup() {

  //initialize the serial port on USART0:
  U0init(9600);
  //reads the current time
  RTC.read(tm);
  //get seconds from initial start to display every minute
  newSecond = tm.Second;

  //servo motor setup
  myStepper.setSpeed(stepperMSpeed);
  //setup for fan motor
  //pinMode(speedPin, OUTPUT);
  *ddr_d |= 0x08;
  //pinMode(dir1, OUTPUT);
  *ddr_g |= 0x20;
  //pinMode(dir2, OUTPUT);
  *ddr_e |= 0x28;


  //set up Red LED, set PL7 as output 
  *ddr_l |= 0xA0;

  //set up green LED, set PG1 as output
  *ddr_g |= 0x02;   

  //set up Yellow LED, set PD7 as output
  *ddr_d |= 0x80;
  //set up button as PB0 input and PB2 as input
  *ddr_b &= 0xFA;

  //set buttons for stepper motor
  *ddr_b &= 0x3F;

  //setup timers
  *myTCCR1A = 0x00; //normal mode
  *myTCCR1B = 0x00; // no prescaling
  *myTCCR1C = 0x00;
  *myTIMSK1 |= 0x01;

  *ddr_a |= 0x01;
  *port_a |= 0x00;
  //analogWrite(22, 0);
  //analogWrite(6,0);
  HT.begin();
  lcd.begin(16,2);
  //pinMode(POWER_PIN, OUTPUT);   // configure D7 pin as an OUTPUT
  *ddr_h |= 0x10;
  //digitalWrite(POWER_PIN, LOW); // turn the sensor OFF
  *port_h &= 0xEF;

  adc_init();
}

void loop() {
  
  //displays time and date every minute
  if(state != 0 && (newMinute != tm.Minute && newSecond == tm.Second))
  {
    newMinute = tm.Minute;
    //change this line to lcd info 
    timeDisplay();
    
  }

  readTime = RTC.read(tm);

  //on/off button pressed
  if(*pin_b & 0x01)
  { 
    *myTCCR1B |= 0x01;
    while(*pin_b & 0x01){};
  }

    

  switch(state)
  {
    case 1:
      state = IDLE();
      break;

    case 2:
      state = ERROR();
      break;

    case 3:
      state = RUNNING();
      break;
      
    case 0:
      state = DISABLED();
      break; 

    default:
      break;

  }
}

ISR(TIMER1_OVF_vect)
{
  // stop the timer
  *myTCCR1B &= 0xF8;
  // Load the Count
  *myTCNT1 = 0;

  state = !state;
}

void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}


int DISABLED()
{
  //turn red LED OFF
  *port_l &= 0x7F;
  //turn blue LED off
  *port_l &= 0xDF;
  //turn Yellow LED ON 
  *port_d |= 0x80;
  //turn Green LED OFF
  *port_g &= 0xFD;

  //stop fan motor
  *port_e &= 0xF7;

  if(!OFFFlag)
  {
    //Serial.print("Motor OFF ");
    U0putchar('M');
    U0putchar('o');
    U0putchar('t');
    U0putchar('o');
    U0putchar('r');
    U0putchar(' ');
    U0putchar('O');
    U0putchar('F');
    U0putchar('F');
    U0putchar(' ');
    
    timeDisplay();
    OFFFlag = 1;
  }
   
   //step motor code 
  if(*pin_b & 0x80)
  {
    
    if(!rightFlag)
    {
      printStepperRotation(0);
      timeDisplay();

      //leftFlag = 1;

    }
    //stpper motor rotates counterclockwise while button is pressed  
    while(*pin_b & 0x80){myStepper.step(-5);};
  }
  if(*pin_b & 0x40)
  {
    //displays each time the button is pressed
    if(!leftFlag)
    {
      printStepperRotation(1);
      timeDisplay();

      leftFlag = 1;

     
      leftFlag = 0;

    }
    //stpper motor rotates clockwise while button is pressed  
    while(*pin_b & 0x40){myStepper.step(5);};
  } 

  lcd.clear();

  lowFlag = 0;
  medFlag = 0;
  highFlag = 0;
  errorFlag = 0;
  leftFlag = 0;
  rightFlag = 0;
  //stop fan motor
  *port_e &= 0xF7;

  return 0;
}

int IDLE()
{
  //Exact time stamp (using real time clock) should record transition times

  //turn red LED OFF
  *port_l &= 0x7F;
  //turn blue LED off
  *port_l &= 0xDF;
  //turn Yellow LED OFF
  *port_d &= 0x7F;
  //turn Green LED on
  *port_g |= 0x02;

  //stop fan motor
  *port_e &= 0xF7;


  //step motor code
  if(*pin_b & 0x80)
  {
    
    if(!rightFlag)
    {
      printStepperRotation(0);
      timeDisplay();

      //leftFlag = 1;
      //leftFlag = 0;

    }  
    while(*pin_b & 0x80){myStepper.step(-5);};
  }
  if(*pin_b & 0x40)
  {
    if(!leftFlag)
    {
      printStepperRotation(1);
      timeDisplay();

      //rightFlag = 1;

     
      //rightFlag = 0;

    }
    
    while(*pin_b & 0x40){myStepper.step(5);};
  }


  //digitalWrite(POWER_PIN, HIGH);  // turn the sensor ON
  *port_h |= 0x10;
  //value = analogRead(SIGNAL_PIN); // read the analog value from sensor
  value = adc_read(SIGNAL_PIN);

  //value = 241;
  
  //digitalWrite(POWER_PIN, LOW);   // turn the sensor OFF
  *port_h &= 0xEF;

  LCDDisplayInfo(value);

  double t = HT.readTemperature(true); // 71.96 F

  //return to go to Error state if water level too low
  if(value <= 240)
  {     
    lowFlag = 1;
    medFlag = 0;
    highFlag = 0;
    ONFlag = 0;
    OFFFlag = 0;
    leftFlag = 0;
    rightFlag = 0;
    return 2;
  } 
  else if(value > 240 && t > 60) 
  {
    lowFlag = 0;
    medFlag = 1;
    highFlag = 0;
    ONFlag = 0;
    OFFFlag = 0;
    leftFlag = 0;
    rightFlag = 0;
    return 3;
  }
  else 
  {
    //come back to this state if nothing changes.
    return 1;
  }
}

int ERROR()
{
  //turn blue LED off
  *port_l &= 0xDF;
  //turn Green LED OFF
  *port_g &= 0xFD;
  //turn Yellow LED OFF
  *port_d &= 0x7F;
  //turn red LED on 
  *port_l |= 0x80;

  //stop fan motor
  *port_e &= 0xF7;

  if(!OFFFlag)
  {
    //Serial.print("Motor OFF ");
    U0putchar('M');
    U0putchar('o');
    U0putchar('t');
    U0putchar('o');
    U0putchar('r');
    U0putchar(' ');
    U0putchar('O');
    U0putchar('F');
    U0putchar('F');
    U0putchar(' ');
    timeDisplay();     
  }
  

  //stop the step motor
  myStepper.step(0);

  if(!errorFlag)
  {
    errorFlag = 1;
    lcd.clear();
    lcd.print("Water level Low");
  }
  
  while(!(*pin_b & 0x04)){
    //if the user enters wants to turn off the system
    if(*pin_b & 0x01)
    { 
      *myTCCR1B |= 0x01;
      while(*pin_b & 0x01){};
      return 0;
    }  
    
  }
  while(*pin_b & 0x04){};
  errorFlag = 0;
  lowFlag = 0;
  medFlag = 0;
  highFlag = 0;
  leftFlag = 0;
  rightFlag = 0;

  //return idle state;  
  return 1;
}

int RUNNING()
{
   //turn Green LED OFF
  *port_g &= 0xFD;
  //turn Yellow LED OFF
  *port_d &= 0x7F;
  //turn red LED OFF
  *port_l &= 0x7F;
  //turn blue LED ON
  *port_l |= 0x20;

  //turn fan on
  //digitalWrite(dir1, HIGH);
  *port_g |= 0x20;
  //digitalWrite(dir2, LOW);
  *port_e &= 0xDF;
  //analogWrite(5, 255);
  *port_e |= 0x08;

  if(!ONFlag)
  {
    //Serial.print("Motor ON ");
    U0putchar('M');
    U0putchar('o');
    U0putchar('t');
    U0putchar('o');
    U0putchar('r');
    U0putchar(' ');
    U0putchar('O');
    U0putchar('N');
    U0putchar(' ');

    timeDisplay(); 
    ONFlag = 1;
  }
  

  //stepper motor code here
  if(*pin_b & 0x80)
  {
    
    if(!rightFlag)
    {
      printStepperRotation(0);
      timeDisplay();

      //leftFlag = 1;

    }  
    while(*pin_b & 0x80){myStepper.step(-5);};
  }
  if(*pin_b & 0x40)
  {
    if(!leftFlag)
    {
      printStepperRotation(1);
      timeDisplay();

      leftFlag = 1;
     
      leftFlag = 0;

    }
    
    while(*pin_b & 0x40){myStepper.step(5);};
  }

  double t = HT.readTemperature(true); // 71.96 F
  if(t < 60) // if the temperature is below 70, go to IDLE state.
  {
    lowFlag = 0;
    medFlag = 0;
    highFlag = 0;
    ONFlag = 0;
    OFFFlag = 0;
    leftFlag = 0;
    rightFlag = 0;
    return 1;
  }

  //digitalWrite(POWER_PIN, HIGH);  // turn the sensor ON
  *port_h |= 0x10;
  //value = analogRead(SIGNAL_PIN); // read the analog value from sensor
  value = adc_read(SIGNAL_PIN);

  //value = 241;
  //digitalWrite(POWER_PIN, LOW);   // turn the sensor OFF
  *port_h &= 0xEF;
  LCDDisplayInfo(value);

  //return to go to Error state if water level too low
  if(value <= 240)
  {
    OFFFlag = 0;
    ONFlag = 0;
    lowFlag = 1;
    medFlag = 0;
    highFlag = 0;
    leftFlag = 0;
    rightFlag = 0;
    return 2;
  } 

  //if the user wants to turn the sys off
  if(*pin_b & 0x01)
  { 
    while(*pin_b & 0x01){};
    OFFFlag = 0;
    ONFlag = 0;
    lowFlag = 0;
    medFlag = 0;
    highFlag = 0;
    leftFlag = 0;
    rightFlag = 0;
    return 0;
  } 

  // come back to running state
  return 3;
}

void LCDDisplayInfo(int value)
{
    if(value > 240 && value <= 290)
    {
      if(!medFlag)
      {
        lcd.clear();
        //Serial.println(value);
        float h = HT.readHumidity();
        lcd.print("H=");
        lcd.print(h);
        lcd.setCursor(0, 1);
        float t = HT.readTemperature(true);
        lcd.print("T=");
        lcd.print(t);
        //lcd.clear();
        //lcd.print("med "); 
        //lcd.print(value);
        lowFlag = 0;
        medFlag = 1;
        highFlag = 0;
  
      }
      
    }
    else if(value > 290)
    {
      if(!highFlag)
      {
        // Serial.println(value);
        lcd.clear();
        //lcd.print("HIGH ");
        //lcd.print(value); 
        float h = HT.readHumidity();
        lcd.print("H=");
        lcd.print(h);
        lcd.setCursor(0, 1);
        float t = HT.readTemperature(true);
        lcd.print("T=");
        lcd.print(t);
        //lcd.print("high ");
        //lcd.print(value);   
        lowFlag = 0;
        medFlag = 0;
        highFlag = 1;
      }
    }
}

void timeDisplay()
{
  if (RTC.read(tm)) {
    printTimeInfo();
  }   
}

void printTimeInfo()
{
  //Serial.print("Time = (Hours:minutes:seconds)");
  U0putchar('T');
  U0putchar('i');
  U0putchar('m');
  U0putchar('e');
  U0putchar(' ');
  U0putchar('=');
  U0putchar(' ');

  print2digits(tm.Hour);
  U0putchar(':');
  print2digits(tm.Minute);
  U0putchar(':');
  print2digits(tm.Second);
  //Serial.print(", Date (D/M/Y) = ");
  U0putchar(',');
  U0putchar(' ');
  U0putchar('D');
  U0putchar('a');
  U0putchar('t');
  U0putchar('e');
  U0putchar(' ');
  U0putchar('=');
  U0putchar(' ');
  print2digits(tm.Month);
  U0putchar('/');
  print2digits(tm.Day);
  U0putchar('/');
  print4digits(tmYearToCalendar(tm.Year));
  U0putchar('\n');
}

void print2digits(int number)
{
  unsigned char firstPart = (number/10) + '0';
  U0putchar(firstPart);
  unsigned char lastPart = (number%10) + '0';
  U0putchar(lastPart);
}

void print4digits(int number)
{
  unsigned char var;
  var = (number / 1000) +'0';
  U0putchar(var);
  number %= 1000;
  var = (number /100) + '0';
  U0putchar(var);
  number %= 100;
  var = (number / 10) +'0';
  U0putchar(var);
  var = (number % 10) + '0';
  U0putchar(var);
}

void printStepperRotation(bool clockwise)
{
  if(!clockwise)
  {
    //Serial.Print("turning counterclockwise ");
    U0putchar('t');
    U0putchar('u');
    U0putchar('r');
    U0putchar('n');
    U0putchar('i');
    U0putchar('n');
    U0putchar('g');
    U0putchar(' ');
    U0putchar('c');
    U0putchar('o');
    U0putchar('u');
    U0putchar('n');
    U0putchar('t');
    U0putchar('e');
    U0putchar('r');
    U0putchar('c');
    U0putchar('l');
    U0putchar('o');
    U0putchar('c');
    U0putchar('k');
    U0putchar('w');
    U0putchar('i');
    U0putchar('s');
    U0putchar('e');
    U0putchar(' ');
  }
  else
  {
    //Serial.Print("turning clockwise ");
    U0putchar('t');
    U0putchar('u');
    U0putchar('r');
    U0putchar('n');
    U0putchar('i');
    U0putchar('n');
    U0putchar('g');
    U0putchar(' ');
    U0putchar('c');
    U0putchar('l');
    U0putchar('o');
    U0putchar('c');
    U0putchar('k');
    U0putchar('w');
    U0putchar('i');
    U0putchar('s');
    U0putchar('e');
    U0putchar(' ');
  }
}

void U0init(unsigned long U0baud)
{
  //initialization code for the ATmega2560 USART0
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

// Read USART0 RDA status bit and return non-zero true if set
unsigned char U0kbhit()
{
  return (*myUCSR0A & RDA);
}

// Wait for USART0 TBE to be set then write character to transmit buffer
void U0putchar(unsigned char U0pdata)
{
  while((*myUCSR0A & TBE) == 0){};
  *myUDR0 = U0pdata;
}
