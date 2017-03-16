
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define HC595_PORT   PORTD
#define HC595_DDR    DDRD

#define HC595_DS_POS PB3    //Data pin (DS) pin location

#define HC595_SH_CP_POS PB5      //Shift Clock (SH_CP) pin location
#define HC595_ST_CP_POS PB4      //Store Clock (ST_CP) pin location

void Init();
void Pulse();
void Latch();
void Wait();
void Write(uint8_t data);

#include <LiquidCrystal.h>
LiquidCrystal lcd(8,9,10,11,12,13);

//#include "delay.h"
//#include "led.h"
//#include "timer.h"
//#include "shift_register.h"

#define PIN_7 0x80
#define LEDDIR DDRD
#define LEDPORT PORTD


volatile int count=0;
volatile int signal_length = 0;

long int RPM_value=0;

void LedOn(int pin);
void LedOff(int pin);
void LedInit(int pin);
void interrupt_innit();
int RPM();
void timer0_setup();

ISR(INT0_vect) {
    signal_length=count;
    count=0;
         }

ISR(TIMER0_COMPA_vect){//timer0 interrupt
// do something
  //LedOn(13);
  count++;
}

int main(){
  lcd.begin(16, 2);
  lcd.clear();
  //lcd.print("RPM:");
  
  timer0_setup();
  interrupt_innit();
     uint8_t led_pattern[9]={
                          0b00000000,
                          0b00000001,
                          0b00000011,
                          0b00000111,
                          0b00001111,
                          0b00011111,
                          0b00111111,
                          0b01111111,
                          0b11111111,
                       };

     //Initialize HC595 system
    Init();
    // Initialize Led
    LEDDIR|=PIN_7; 
   int data=0;
   int led=0;

  while(1){
    RPM();
    //round to nearest 100
    RPM_value =(RPM_value + 50) / 100 * 100;
    
    //Wait();
    //Setting blue lights

        if (RPM_value>12000){
          data=8;
          led=1;
          lcd.setCursor(1,6);
        }
        else if (RPM_value>11000){
          data=7;
          led=1;
          lcd.setCursor(1,6);
            }
        else if (RPM_value>=10000){
          data=6;
          led=1;
          lcd.setCursor(1,6);
            }
        //Setting yellow lights
        else if (RPM_value>8334){
         data=5;
         led=1;
         
         lcd.setCursor(2,6);
        }
        else if (RPM_value>6667){
        data=4;
        led=1;
        lcd.clear();
        lcd.setCursor(2,6);
        }
        else if(RPM_value>5000){
         data=3;
         led=1;
         lcd.clear();
         lcd.setCursor(2,6);
        }
        //Setting red lights
        else if (RPM_value>3334){
         data=2;
         led=1;
         lcd.clear();
         lcd.setCursor(2,6);
        }
        else if  (RPM_value>1667){
        data=1;
        led=1;
        lcd.clear();
        lcd.setCursor(2,6);
        }
        else {
         data=0;
         led=0;
         lcd.clear();
         lcd.setCursor(2,6);
        }

     Write(led_pattern[data]);
     if(led){
      LEDPORT |=PIN_7;
     }
     else{
      LEDPORT&=~PIN_7;
     }
  lcd.print("RPM:");
  lcd.print(RPM_value);  
  }
  return 0;
}


void interrupt_innit() {
  cli();
  // attachInterrupt(0, rising, RISING); in setup
   DDRD &= ~(1 << DDD2);     // Clear the PD2 pin
      // PD2 (INT0 pin) is now an input

      EICRA |= ((1 << ISC00)| (1 << ISC01)); // set INT0 to trigger on rising edge
      EIMSK |= (1 << INT0);     // Turns on INT0

  sei();    // enable global interrupt
}

int RPM() {
  RPM_value=42*60000/(signal_length);
  return RPM_value;
}


void timer0_setup(){
cli();//stop interrupts


  TCCR0A = 0;// set entire TCCR0A register to 0
  TCCR0B = 0;// same for TCCR0B
  TCNT0  = 0;//initialize counter value to 0

  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler. 16Mhz/64 = 250 kHz => 4µs
  TCCR0B |= (1 << CS01) | (1 << CS00);
  // enable timer compare interrupt

  OCR0A = 11;// set compare value 12*4µs=0.048ms

  //enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);

  sei();
}





/***************************************

Configure Connections

****************************************/



//Initialize HC595 System

void Init()
{
   //Make the Data(DS), Shift clock (SH_CP), Store Clock (ST_CP) lines output
   HC595_DDR|=((1<<HC595_SH_CP_POS)|(1<<HC595_ST_CP_POS)|(1<<HC595_DS_POS));
}


//Low level macros to change data (DS)lines
#define DataHigh() (HC595_PORT|=(1<<HC595_DS_POS))

#define DataLow() (HC595_PORT&=(~(1<<HC595_DS_POS)))

//Sends a clock pulse on SH_CP line
void Pulse()
{
   //Pulse the Shift Clock

   HC595_PORT|=(1<<HC595_SH_CP_POS);//HIGH

   HC595_PORT&=(~(1<<HC595_SH_CP_POS));//LOW

}

//Sends a clock pulse on ST_CP line
void Latch()
{
   //Pulse the Store Clock

   HC595_PORT|=(1<<HC595_ST_CP_POS);//HIGH
   _delay_loop_1(1);

   HC595_PORT&=(~(1<<HC595_ST_CP_POS));//LOW
   _delay_loop_1(1);
}


/*

Main High level function to write a single byte to
Output shift register 74HC595.

Arguments:
   single byte to write to the 74HC595 IC

Returns:
   NONE

Description:
   The byte is serially transfered to 74HC595
   and then latched. The byte is then available on
   output line Q0 to Q7 of the HC595 IC.

*/
void Write(uint8_t data)
{
   //Send each 8 bits serially

   //Order is MSB first
   for(uint8_t i=0;i<8;i++)
   {
      //Output the data on DS line according to the
      //Value of MSB
      if(data & 0b10000000)
      {
         //MSB is 1 so output high

         DataHigh();
      }
      else
      {
         //MSB is 0 so output high
         DataLow();
      }

      Pulse();  //Pulse the Clock line
      data=data<<1;  //Now bring next bit at MSB position

   }

   //Now all 8 bits have been transferred to shift register
   //Move them to output latch at one
   Latch();
}

/*

Simple Delay function approx 0.5 seconds

*/

void Wait()
{
   for(uint8_t i=0;i<30;i++)
   {
      _delay_loop_2(0);
   }
}



