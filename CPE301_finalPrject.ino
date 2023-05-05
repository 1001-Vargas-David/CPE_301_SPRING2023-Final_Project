#include <LiquidCrystal.h>
#include <DHT.h>
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



/*volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;*/
//adc pointers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

#define POWER_PIN 7
#define SIGNAL_PIN 5

#define DHT11_PIN 6

int value = 0; // variable to store the sensor value
int newValue = 0;
bool lowFlag = 0;
bool medFlag = 0;
bool highFlag = 0;

DHT HT(DHT11_PIN, DHT11);
LiquidCrystal lcd(24, 26, 28, 30, 32, 34);

void setup() {
  Serial.begin(9600);
  analogWrite(22, 0);
  //analogWrite(6,0);
  HT.begin();
  lcd.begin(16,2);
  //pinMode(POWER_PIN, OUTPUT);   // configure D7 pin as an OUTPUT
  *ddr_h |= 0x10;
  //digitalWrite(POWER_PIN, LOW); // turn the sensor OFF
  *port_h &= 0xEF;

  adc_init();
  //lcd.print("Sensor: ");
  lcd.clear();
  //lcd.print("hello");
}

void loop() {
  //digitalWrite(POWER_PIN, HIGH);  // turn the sensor ON
  *port_h |= 0x10;
  my_delay(10000);                      // wait 10 milliseconds
  //value = analogRead(SIGNAL_PIN); // read the analog value from sensor
  value = adc_read(SIGNAL_PIN);
  //digitalWrite(POWER_PIN, LOW);   // turn the sensor OFF
  *port_h &= 0xEF;

  //lcd.clear();
  //lcd.print(value);
  if(value <= 160)
  {
    if(!lowFlag)
    {
      lcd.clear();
      newValue = value;
     // Serial.println(value);
      float h = HT.readHumidity();
      lcd.print("H = ");
      lcd.print(h);
      lcd.setCursor(0, 1);
      double t = HT.readTemperature(true);
      lcd.print("T = ");
      lcd.print(t);
      //lcd.clear();
      lcd.print(" ");
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
      lcd.clear();
      newValue = value;
      //Serial.println(value);
      float h = HT.readHumidity();
      lcd.print("H = ");
      lcd.print(h);
      lcd.setCursor(0, 1);
      float t = HT.readTemperature(true);
      lcd.print("T = ");
      lcd.print(t);
      //lcd.clear();
      lcd.print(" "); 
      lcd.print(value);
      lowFlag = 0;
      medFlag = 1;
      highFlag = 0;      
    }
    
  }
  else if(value > 165 && value <= 170)
  {
    if(!highFlag)
    {
      newValue = value;
     // Serial.println(value);
      lcd.clear();
      //lcd.print("HIGH ");
     // lcd.print(value); 
      float h = HT.readHumidity();
      lcd.print("H = ");
      lcd.print(h);
      lcd.setCursor(0, 1);
      float t = HT.readTemperature(true);
      lcd.print("T = ");
      lcd.print(t);
      Serial.print(" ");
      Serial.print(value);   
      lowFlag = 0;
      medFlag = 0;
      highFlag = 1;   
      
    }
  }
  //Serial.println(value);
  //lcd.clear();
  /*float h = HT.readHumidity();
  lcd.print("Humidity = ");
  lcd.print(h);
  float t = HT.readTemperature(true);
  lcd.print(" Temperature = ");
  lcd.println(t);*/

  //my_delay(10000);*/
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



/*
//stepper motor based on button input
if(digitalRead(2) == HIGH){
    //Rotate CW slowly at 5 RPM
    myStepper.setSpeed(7);
    //Rotates clockwise if output is high, otherwise stops
    myStepper.step(10);
  }
  else if(digitalRead(5) == HIGH){
    //Rotate CW slowly at 5 RPM
    myStepper.setSpeed(7);
    //Rotates counterclockwise if output is high, otherwise stops
    myStepper.step(-10);
  }
  else{
    //*portH &= 0xFC;
  }
*/
