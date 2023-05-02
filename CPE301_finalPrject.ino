#include <LiquidCrystal.h>
#include <Stepper.h>
//timers for my delay:
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


//UART Pointers
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

//adc pointers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

#define POWER_PIN  7
#define SIGNAL_PIN 5

int value = 0; // variable to store the sensor value
int newValue = 0;
bool lowFlag = 0;
bool medFlag = 0;
bool highFlag = 0;
const int stepsPerRevolution = 2038; //steps for stepper motor

Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11);
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup() {
  U0Init(9600);
  analogWrite(6,0);
  lcd.begin(16,2);
  //pinMode(POWER_PIN, OUTPUT);   // configure D7 pin as an OUTPUT
  *ddr_h |= 0x10;
  //digitalWrite(POWER_PIN, LOW); // turn the sensor OFF
  *port_h &= 0xEF;

  adc_init();
  //lcd.print("Sensor: ");
  
  /*
  // set PH0, PH1, PH3 to output
  *portDDRH |= 0x0B;
  // set PH0, PH1, PH3 LOW
  *portH &= 0xF4;
  */
}

void loop() {
  //digitalWrite(POWER_PIN, HIGH);  // turn the sensor ON
  *port_h |= 0x10;
  my_delay(10000);                      // wait 10 milliseconds
  //value = analogRead(SIGNAL_PIN); // read the analog value from sensor
  value = adc_read(SIGNAL_PIN);
  //digitalWrite(POWER_PIN, LOW);   // turn the sensor OFF
  *port_h &= 0xEF;
  
  
  /*
  //0xF7 rotate counter clockwise, 0xFD clockwise;, LSB (i.e PH0) is enable
  *portH |= 0xFC;
  if(digitalRead(Pin5) == HIGH){
    //Rotate CW slowly at 5 RPM
    myStepper.setSpeed(5);
    //Rotates clockwise if output is high, otherwise stops
    myStepper.step(1);
  }
  */

  if(value <= 160)
  {
    if(!lowFlag)
    {
      newValue = value;
     // Serial.println(value);
      lcd.clear();
      lcd.print("LOW ");
      lcd.print(value);
      lowFlag = 1;
      medFlag = 0;
      highFlag = 0;
    }
     
  }
  else if(value > 160 && value <= 165)
  {
    if(!medFlag)
    {
      newValue = value;
      //Serial.println(value);
      lcd.clear();
      lcd.print("MED "); 
      lcd.print(value);
      lowFlag = 0;
      medFlag = 1;
      highFlag = 0;      
    }
    
  }
  else if (value > 165 && value <= 170)
  {
    if(!highFlag)
    {
      newValue = value;
     // Serial.println(value);
      lcd.clear();
      lcd.print("HIGH ");
      lcd.print(value); 
      lowFlag = 0;
      medFlag = 0;
      highFlag = 1;      
    }
  }
  Serial.println(value);
  //lcd.clear();

  my_delay(10000);
}

void U0Init(int U0baud)
{
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  // Same as (FCPU / (16 * U0baud)) - 1;
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0  = tbaud;
}

void my_delay(unsigned int freq)
{
  // calc period
  double period = 1.0/double(freq);
  // 50% duty cycle
  double half_period = period/ 2.0f;
  // clock period def
  double clk_period = 0.0000000625;
  // calc ticks
  unsigned int ticks = half_period / clk_period;
  // stop the timer
  *myTCCR1B &= 0xF8;
  // set the counts
  *myTCNT1 = (unsigned int) (65536 - ticks);
  // start the timer
  *myTCCR1B |= 0b00000001;
  // wait for overflow
  while((*myTIFR1 & 0x01)==0); // 0b 0000 0000
  // stop the timer
  *myTCCR1B &= 0xF8;   // 0b 0000 0000
  // reset TOV           
  *myTIFR1 |= 0x01;
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


#include <DHT.h>
//#include <DHT_U.h>

volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;

#define DHT11_PIN 7

DHT HT(DHT11_PIN, DHT11);



void setup(){
  *myTCCR1A = 0x00; //normal mode
  *myTCCR1B = 0x01; // no prescaling
  *myTCCR1C = 0x00; //no force output compare
  *myTIFR1 = 0x01;

  Serial.begin(9600);
  HT.begin();
}

void loop(){
  Serial.print("before ");
  Serial.println(*myTIFR1, HEX);
  float h = HT.readHumidity();
  Serial.print("Humidity = ");
  Serial.print(h);
  float t = HT.readTemperature(true);
  Serial.print(" Temperature = ");
  Serial.println(t);
  my_delay(1000);
}

/*
//NOTE: SET PIN 5 and 6 to input in setup function
 //0xF7 rotate counter clockwise, 0xFD clockwise;, LSB (i.e PH0) is enable
  //*portH |= 0xFD;
  if(digitalRead(Pin5) == HIGH){
    //*portH |= 0xFD;
    //Rotate CW slowly at 5 RPM
    myStepper.setSpeed(5);
    //Rotates clockwise if output is high, otherwise stops
    myStepper.step(10);
  }
  else if(digitalRead(6) == HIGH){
    //Rotate CW slowly at 5 RPM
    myStepper.setSpeed(5);
    //Rotates clockwise if output is high, otherwise stops
    myStepper.step(-10);
  }
  else{
    //*portH &= 0xFC;
  }
*/
