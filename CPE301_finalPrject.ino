#include <LiquidCrystal.h>
#include <DHT.h>
#include <Stepper.h>
#include <Wire.h>
#include <TimeLib.h>
#include <DS1307RTC.h>

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



/*
// UART Pointers
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
*/

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
int testing = 5;
int value = 0; // variable to store the sensor value
volatile bool isActive = 0;
bool timerRunning = 0;
volatile bool lowFlag = 0;
volatile bool medFlag = 0;
volatile bool highFlag = 0;
volatile bool fanOn = 0;

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

  Serial.begin(9600);
  //set up green LED, set PG1
  *ddr_g |= 0x02;   

  //set up Yellow LED, set PD7 as output
  *ddr_d |= 0x80;
  //set up button as PB0 input
  *ddr_b &= 0xFE;

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
  
  //button pressed
  if(*pin_b & 0x01)
  { 

    Serial.println("inside if ");
    *myTCCR1B |= 0x01;
    while(*pin_b & 0x01){};
  }

  if(!isActive)
  {
    //turn Yellow LED ON 
    *port_d |= 0x80;
    //turn Green LED OFF
    *port_g &= 0xFD;
    lcd.clear();
    Serial.print("isActive is ");
    Serial.println(isActive);
    lowFlag = 0;
    medFlag = 0;
    highFlag = 0;
    //stop fan motor
    *port_e &= 0xF7;
   // myStepper.step(0);
    fanOn = 0;
  }
  else
  {
    Serial.print("isActive is ");
    Serial.println(isActive);

    //turn fan on
    //digitalWrite(dir1, HIGH);
    *port_g |= 0x20;
    //digitalWrite(dir2, LOW);
    *port_e &= 0xDF;
    //analogWrite(5, 255);
    *port_e |= 0x08;


    myStepper.step(5);
    //turn Yellow LED OFF
    *port_d &= 0x7F;
    //turn Green LED on
    *port_g |= 0x02;
    //digitalWrite(POWER_PIN, HIGH);  // turn the sensor ON
    *port_h |= 0x10;
    //myStepper.step(stepsPerRevolution);
    //value = analogRead(SIGNAL_PIN); // read the analog value from sensor
    value = adc_read(SIGNAL_PIN);
    //digitalWrite(POWER_PIN, LOW);   // turn the sensor OFF
    *port_h &= 0xEF;

    if(value <= 240)
    {
      if(!lowFlag)
      {
        //lcd.clear();
        // Serial.println(value);
        float h = HT.readHumidity();
        lcd.print("H=");
        lcd.print(h);
        lcd.setCursor(0, 1);
        double t = HT.readTemperature(true);
        lcd.print("T=");
        lcd.print(t);
        //lcd.clear();
        lcd.print("low ");
        lcd.print(value);
        lowFlag = 1;
        medFlag = 0;
        highFlag = 0;
      }
      
    }
    else if(value > 240 && value <= 290)
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
        lcd.print("med "); 
        lcd.print(value);
        lowFlag = 0;
        medFlag = 1;
        highFlag = 0;      
      }
      
    }
    else if(value > 290 && value <= 300)
    {
      if(!highFlag)
      {
      // Serial.println(value);
        lcd.clear();
        //lcd.print("HIGH ");
      // lcd.print(value); 
        float h = HT.readHumidity();
        lcd.print("H=");
        lcd.print(h);
        lcd.setCursor(0, 1);
        float t = HT.readTemperature(true);
        lcd.print("T=");
        lcd.print(t);
        lcd.print("high ");
        lcd.print(value);   
        lowFlag = 0;
        medFlag = 0;
        highFlag = 1;   
        
      }
    }
  }
  
}

ISR(TIMER1_OVF_vect)
{
  // stop the timer
  *myTCCR1B &= 0xF8;
  // Load the Count
  *myTCNT1 = 0;

  isActive = !isActive;
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


void DISABLED()
{


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

// Read USART0 RDA status bit and return non-zero true if set
unsigned char U0kbhit(){
  return (*myUCSR0A & RDA);
}

// Wait for USART0 TBE to be set then write character to
// transmit buffer
void U0putchar(unsigned char U0pdata){
  while((*myUCSR0A & TBE) == 0){};
    *myUDR0 = U0pdata;
}
