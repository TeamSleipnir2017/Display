
#include <avr/io.h>
#include <avr/interrupt.h>

#include <LiquidCrystal.h> // This allows us to see the RPM values displayed on 
//the screen since the programming of the LCD Library was not completed

//Define for shift register

#define Clock_PORT   PORTB //Shift and Store clock connected to PORTB
#define Clock_DDR    DDRB

#define Data_PORT PORTD //Data pin connected to PORTD
#define Data_DDR DDRD

#define SH_CP PB4      //Shift Clock (SH_CP)  
#define ST_CP PB5      //Store Clock (ST_CP)
#define Data PD3   //Data pin (DS)

// Define LCD:
LiquidCrystal lcd(9, 8, 7, 6, 5, 4);

//Define LED
#define PIN_10 0x04
#define LEDDIR DDRB
#define LEDPORT PORTB


volatile int count = 0; // Counter used to measure signal length
volatile int signal_length = 0; //Variable that stores value of signal length
long int RPM_value = 0; // Variable that stores value of RPM

//Function defenitions

//Shift Register
void SRInit(); // Function that Initializes Shiftregister (SR)
void Pulse(); // Function that pulses Shift Clock
void Latch(); // Function that pulses Store Clock
void Write(uint8_t data); // Function that sends binary data to SR

//LED
void LedInit(); //Initialize LED
void LedOn(); // Turn LED on
void LedOff(); // Turn LED off
void display_lights(); // Control LEDs to show optimal gear shifting

//Signal measurements
void InterruptInit(); // Initialize interrupt INT0
void Timer0Init(); // Initialize timer0
void RPM(); // Calculate RPM

//LCD
void print_RPM();
void LCDInit();


//
int main() {

  //Initialize LCD
  LCDInit();

  //Initialize TIMER0
  Timer0Init();

  //Initialize INT0
  InterruptInit();

  //Initialize HC595 system
  SRInit();

  // Initialize Led
  LedInit();



  while (1) {
    //Calculates RPM from signal
    RPM();
    //Turn on LEDs for gearshifting
    display_lights();
    //Display RPM on screen
    print_RPM();
  }
  return 0;
}
/***************************************
  Interrupt and timer functions

****************************************/

// External interrupt INT0
ISR(INT0_vect) {
  signal_length = count; //
  count = 0; // reset timer
}

//timer0 interrupt
ISR(TIMER0_COMPA_vect) {
  count++; // Measure time between signals
}

void InterruptInit() {
  cli();

  DDRD &= ~(1 << DDD2);     // Clear the PD2 pin
  // PD2 (INT0 pin) is now an input

  EICRA |= ((1 << ISC00) | (1 << ISC01)); // set INT0 to trigger on rising edge
  EIMSK |= (1 << INT0);     // Turns on INT0

  sei();    // enable global interrupt
}

void RPM() {
  RPM_value = 42 * 60000 / (signal_length);
}


void Timer0Init() {
  cli();//stop interrupts


  TCCR0A = 0;// set entire TCCR0A register to 0
  TCCR0B = 0;// same for TCCR0B
  TCNT0  = 0;//initialize counter value to 0

  TCCR0A |= (1 << WGM01);
  // Set CS01 and CS00 bits for 64 prescaler. 16Mhz/64 =  //250 kHz => 4us
  TCCR0B |= (1 << CS01) | (1 << CS00);
  // enable timer compare interrupt

  OCR0A = 11;// set compare value 12*4us=0.048ms

  //enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);

  sei();
}


/***************************************

  Shift Register functions

****************************************/
#define DataHigh() (Data_PORT|=(1<<Data)) // Set data port high

#define DataLow() (Data_PORT&=(~(1<<Data))) // Set data port low


void SRInit()
{

  Clock_DDR |= ((1 << SH_CP) | (1 << ST_CP)); //Make Shift clock (SH_CP) and Store Clock (ST_CP) lines output

  Data_DDR |= (1 << Data); //Make the Data(DS) line output
}


void Pulse()
{
  //Pulse the Shift Clock

  Clock_PORT |= (1 << SH_CP); //HIGH

  Clock_PORT &= ~(1 << SH_CP); //LOW

}


void Latch()
{
  //Pulse the Store Clock

  Clock_PORT |= (1 << ST_CP); //HIGH
  Delay_us(1);

  Clock_PORT &= ~(1 << ST_CP); //LOW
  Delay_us(1);
}



void Write(uint8_t data)
{
  //Send 8 bit binary sequence

  //Start with MSB
  for (uint8_t i = 0; i < 8; i++)
  {

    if (data & 0b10000000) // if MSB is 1 then the output should be high
    {

      DataHigh(); // HIGH
    }

    else  // if MSB is 0 then the ouput is low
    {

      DataLow(); // LOW
    }

    Pulse();  //Pulse the Clock line
    data = data << 1; //Shift data so next bit is in MSB position

  }

  //All bits have been shifted in

  Latch(); // Pulse the Store Clock line
}

/***************************************

  Delay functions

****************************************/

void Delay_ms(unsigned int millisek) {
  unsigned volatile long Max, Count;
  Max = 380 * millisek;
  Count = 0;
  while (Count != Max) {
    Count++;
  }
}
void Delay_us(unsigned int microsek) {
  unsigned volatile long Max, Count;
  Max = 0.38 * microsek;
  Count = 0;
  while (Count != Max) {
    Count++;
  }
}
/***************************************

  LED

****************************************/
void LedOn()
{
  LEDPORT |= PIN_10;
}

void LedOff()
{
  LEDPORT &= ~PIN_10;
}

void LedInit()
{
  LEDDIR |= PIN_10;
}

void display_lights() {
  // Variable that keeps value 0/1 if Led connected to pin 10 is off/on
  int led = 0;


  //Variable that keeps value of indices of led_pattern array displayed by the LEDs
  uint8_t pattern;

  // Turns LEDs on and off depending on binary value 0=off, 1=off.
  // Each value references an LED connected to a different output of the shift register
  //(Q7, Q6, Q5, Q4, Q3, Q2, Q1, Q0)
  uint8_t led_pattern[9] = {
    0b00000000,
    0b00000010,
    0b00000110,
    0b00001110,
    0b00011110,
    0b00111110,
    0b01111110,
    0b11111110,
    0b11111111,
  };

  //round to nearest 100
  RPM_value = (RPM_value + 50) / 100 * 100;

  //Setting blue lights (Q0,Q7,Q6)
  if (RPM_value >= 9000) {
    pattern = 8;
    led = 1;
  }
  else if (RPM_value >= 8500) {
    pattern = 7;
    led = 1;
  }
  else if (RPM_value >= 8000) {
    pattern = 6;
    led = 1;
  }
  //Setting yellow lights (Q6,Q5,Q4)
  else if (RPM_value >= 7500) {
    pattern = 5;
    led = 1;
  }
  else if (RPM_value >= 7000) {
    pattern = 4;
    led = 1;
  }
  else if (RPM_value >= 6500) {
    pattern = 3;
    led = 1;
  }
  //Setting red lights (Q2,Q1 and LED connected to pin10)
  else if (RPM_value >= 6000) {
    pattern = 2;
    led = 1;
  }
  else if  (RPM_value >= 5500) {
    pattern = 1;
    led = 1;
  }
  else if  (RPM_value >= 5000) {
    pattern = 0;
    led = 1;
  }
  else {
    pattern = 0;
    led = 0;
  }
  Write(led_pattern[pattern]);
  if (led) {
    LedOn();
  }
  else {
    LedOff();
  }

}
/***************************************

  LCD

****************************************/
void print_RPM() {

  if (RPM_value >= 10000) {
    lcd.setCursor(1, 6);
  }
  else {
    lcd.clear(); //  Clears the first digit from a number previously displayed that was higher than 10 000
    lcd.setCursor(2, 6);
  }

  lcd.print("RPM:");
  lcd.print(RPM_value);
  Delay_ms(10);  // Keeps the LCD screen from flickering
}

void LCDInit() {
  lcd.begin(16, 2);
  lcd.clear();
}

