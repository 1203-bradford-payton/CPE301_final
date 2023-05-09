#include <TimeLib.h>
#include <dht11.h>
#include <LiquidCrystal.h>
#include <Stepper.h>

#define printTimeDate serialDisplay("|%d/%d/%d|%d:%d| ", month(), day(), year(), hour(), minute())

#define DHT11PIN 4
#define LEDPIN 33
#define STEPS 100
#define TEMP_THRESH 20.0

#define yellowLED 32
#define blueLED 33
#define redLED 34
#define greenLED 35
#define fanPin 42

dht11 DHT11;
volatile unsigned int* ADC_DATA = (unsigned int*) 0x78;
volatile unsigned int temp=0, humidity=0;
signed int previous = 0;
volatile byte enables = 0b00000000; //Enables: [temp][display][stop][start][reset][water_level][fan][stepper]
//The enables byte controls when certain conditions can change state and controls the fan and stepper.
//[temp-controls if the temperature reading can change states][display-determines if display will show temp and hummidity values][stop-determines if stop button can change states][start-determines if start button can change states]
//[reset-determines if reset button can change states][water_level-determines if water sensor can change states][fan-toggles fan on and off][stepper-toggles stepper function on and off]

//Set up LCD and stepper
const int rs = 44, en = 45, d0 = 46, d1 = 47, d2 = 48, d3 = 49, d4 = 50, d5 = 51, d6 = 52, d7 = 53;
LiquidCrystal lcd(rs, en, d0, d1, d2, d3, d4, d5, d6, d7);
Stepper stepper(STEPS, 36,37,38,39);

//For some reason, I have to make the prototype for this function manually
void serialDisplay(char input[100], int num0=0, int num1=0, int num2=0, int num3=0, int num4=0, int num5=0);

void setup() {
  setTime(05,13,0,02,05,2023);
  lcd.begin(16, 2);
  stepper.setSpeed(200);

  //Set led pins as output
  DDRC |= 0b00111100;
  //Set fan pin as output
  DDRL |= 0b10000000;

  //Set up interrupts
  attachInterrupt(0, Idle, RISING);
  attachInterrupt(1, Disabled, RISING);
  attachInterrupt(digitalPinToInterrupt(18), Idle, RISING);

  //Initialize serial port and ADC
  U0init(9600);
  adc_init();
 
  //Print first line
  while(!(UCSR0A & (1<<UDRE0)));
  serialDisplay("|  DATE  |TIME|MESSAGE\n");
 
  //Set up comparitor
  ACSR = 0b01011000;
 
  //Set up external interrupts
  EIMSK = 0b00000000;
  EICRA = 0b11110000;
  EICRB = 0b00001111;
 
  //Start program in idle state
  Idle();
}

void loop() {
  //Read temp and hummidity
  int chk = DHT11.read(DHT11PIN);
  humidity = DHT11.humidity;
  temp = DHT11.temperature;

  //Enable/disable interrupts. I made the enables byte so that the middle three bits correspond to their place in the EIMSK, which enables/disables interrupts.
  //Enables: [temp][display][stop][start][reset][water_level][fan][stepper]
  EIMSK |= (enables & 0b00111000);  //Enable interrupts based on enables byte
  EIMSK &= (enables | 0b11000111);  //Disable interrupts based on enables byte
  EIFR = 0b11111111;  //clear interrupts

  //Enable/disable water level interrupt (seperate from the others because it is a comparitor interrupt)
  if(enables & 0b00000100){
    ACSR &= 0b01111111;
  }
  else{
    ACSR |= 0b10000000;
  }

  //Display conditions if enabled
  if(enables & 0b01000000){
    displayConditions(temp,humidity);
  }

  //Run stepper function if enabled
  if(enables & 0b00000001){
    int steps = 0;
    int val = adc_read(0);
    steps = val - previous;
    if(steps > 50){\
      printTimeDate;
      serialDisplay("Moveing stepper %d steps\n", steps);
      stepper.step(steps);
    }
   
    previous = val;
  }

  //Change state based on temp if enabled
  if((enables & 0b10000000) and temp > TEMP_THRESH and enables != 0b11100111){
    Running();
  }
  else if((enables & 0b10000000) and temp < TEMP_THRESH and enables != 0b11100101){
    Idle();
  }

  //Turn on or off fan based on bit in enables
  pinWrite(fanPin, enables & 0b00000010);
 

  delay(500);

}

void Idle(){
  printTimeDate;
  serialDisplay("Entering Idle state\n");
  enables = 0b11100101; //Enables: [temp][display][stop][start][reset][water_level][fan][stepper]
  clearLED();
  pinWrite(greenLED, 1);
}

void Disabled(){
  printTimeDate;
  serialDisplay("Entering Disabled state\n");
  lcd.clear();
  lcd.print("Disabled");
  enables= 0b00010001; //Enables: [temp][display][stop][start][reset][water_level][fan][stepper]
  clearLED();
  pinWrite(yellowLED, 1);
}

void Running(){
  printTimeDate;
  serialDisplay("Entering Running state\n");
  enables = 0b11100111; //Enables: [temp][display][stop][start][reset][water_level][fan][stepper]
  clearLED();
  pinWrite(blueLED, 1);
}

void Error(){
  printTimeDate;
  serialDisplay("Entering Error state\n");
  lcd.clear();
  lcd.print("Water level is");
  lcd.setCursor(0,1);
  lcd.print("too low");
  enables = 0b00101000; //Enables: [temp][display][stop][start][reset][water_level][fan][stepper]
  clearLED();
  pinWrite(redLED, 1);
}

ISR(ANALOG_COMP_vect){
  Error();
}

void displayConditions(int temp,int humidity){
  //Displays temp and humidity on LCD
  lcd.setCursor(0, 0);
  lcd.print("Humidity: ");
  lcd.print((float)humidity, 1);
  lcd.setCursor(0, 1);
  lcd.print("Temp: ");
  lcd.print((float)temp, 1);
}

void clearLED(){
  pinWrite(redLED, 0);
  pinWrite(yellowLED, 0);
  pinWrite(blueLED, 0);
  pinWrite(greenLED, 0);
}

//Initialize serial port
void U0init(unsigned long U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 UCSR0A = 0x20;
 UCSR0B = 0x18;
 UCSR0C = 0x06;
 UBRR0  = tbaud;
}

//Prints string with numbers to the serial monitor. Format works like printf
void serialDisplay(char input[100], int num0=0, int num1=0, int num2=0, int num3=0, int num4=0, int num5=0){
  int numArray[6] = {num0, num1, num2, num3, num4, num5};
  int numCount = 0;
  for(int i=0; i<100 and input[i] != '\0'; i++){
   
    //Print numbers if specified. Format works like printf where %d will become a number
    if(input[i] == '%' and input[i+1] == 'd'){
      unsigned int numLength=1;
      //Find amount of digits in number
      while(numArray[numCount]/exponent(10, numLength) > 0){
        numLength++;
      }
      //Print each digit of number individually
      for(int z=numLength; z>0; z--){
        while((!(UCSR0A & (1<<UDRE0))));
        UDR0 = (numArray[numCount]%exponent(10, z)/exponent(10, z-1))+48;
      }
      numCount ++;
    }
    //Display text part of string. Do dot display the d in %d
    else if(!(input[i] == 'd' and input[i-1] == '%')){
     while((!(UCSR0A & (1<<UDRE0))));
     UDR0 = input[i];
    }
  }
}

int exponent(int base, int power){
  int output=1;
  if(power == 0){
    return(1);
  }
  else{
    for(int i=0; i<power; i++){
      output = base * output;
    }
    return(output);
  }
}

//Turns any pin on or off, works like digitalWrite()
void pinWrite(int pin, bool state){
  //Ports and bits arrays give the port and corresponding bit for a pin. For example, pin 0 is bit 0 in port E
  char ports[53]={"EEEEGEHHHHBBBBJJHHDDDDAAAAAAAACCCCCCCCDGGGLLLLLLLLBBBB"};
  unsigned int bits[54] ={0,1,4,5,5,3,3,4,5,6,4,5,6,7,1,0,1,0,3,2,1,0,0,1,2,3,4,5,6,7,7,6,5,4,3,2,1,0,7,2,1,0,7,6,5,4,3,2,1,0,3,2,1,0};
  //Make two bits based on if we are turning the pin on or off
  char onMask = 0b00000000, offMask = 0b11111111;
  if(state){onMask=0b00000001;offMask=0b11111111;}
  else{onMask=0b00000000;offMask=0b11111110;}
  //Change the appropriate bit in the appropriate port
  switch(ports[pin]){
    case 'A':
        PINA &= (offMask << bits[pin]);
        PINA |= (onMask << bits[pin]);
      break;
 
      case 'B':
        PINB &= (offMask << bits[pin]);
        PINB |= (onMask << bits[pin]);
      break;
 
      case 'C':
        PINC &= (offMask << bits[pin]);
        PINC |= (onMask << bits[pin]);
      break;
 
      case 'D':
        PIND &= (offMask << bits[pin]);
        PIND |= (onMask << bits[pin]);
      break;
 
      case 'E':
        PINE &= (offMask << bits[pin]);
        PINE |= (onMask << bits[pin]);
      break;
     
      case 'G':
        PING &= (offMask << bits[pin]);
        PING |= (onMask << bits[pin]);
      break;

      case 'H':
        PINH &= (offMask << bits[pin]);
        PINH |= (onMask << bits[pin]);
       
      break;

      case 'J':
        PINJ &= (offMask << bits[pin]);
        PINJ |= (onMask << bits[pin]);
      break;

      case 'L':
        PINL &= (offMask << bits[pin]);
        PINL |= (onMask << bits[pin]);
      break;

      default:
        serialDisplay("Error in pinWrite function");
      break;
  }
}

void pinSet(int pin, bool state){
  //0 is input, 1 is output
  //Ports and bits arrays give the port and corresponding bit for a pin. For example, pin 0 is bit 0 in port E
  char ports[53]={"EEEEGEHHHHBBBBJJHHDDDDAAAAAAAACCCCCCCCDGGGLLLLLLLLBBBB"};
  unsigned int bits[54] ={0,1,4,5,5,3,3,4,5,6,4,5,6,7,1,0,1,0,3,2,1,0,0,1,2,3,4,5,6,7,7,6,5,4,3,2,1,0,7,2,1,0,7,6,5,4,3,2,1,0,3,2,1,0};
  //Make two bits based on if we are turning the pin on or off
  char onMask = 0b00000000, offMask = 0b11111111;
  if(state){onMask=0b00000001;offMask=0b11111111;}
  else{onMask=0b00000000;offMask=0b11111110;}
  //Change the appropriate bit in the appropriate port
  switch(ports[pin]){
    case 'A':
        DDRA &= (offMask << bits[pin]);
        DDRA |= (onMask << bits[pin]);
      break;
 
      case 'B':
        DDRB &= (offMask << bits[pin]);
        DDRB |= (onMask << bits[pin]);
      break;
 
      case 'C':
        DDRC &= (offMask << bits[pin]);
        DDRC |= (onMask << bits[pin]);
      break;
 
      case 'D':
        DDRD &= (offMask << bits[pin]);
        DDRD |= (onMask << bits[pin]);
      break;
 
      case 'E':
        DDRE &= (offMask << bits[pin]);
        DDRE |= (onMask << bits[pin]);
      break;
     
      case 'G':
        DDRG &= (offMask << bits[pin]);
        DDRG |= (onMask << bits[pin]);
      break;

      case 'H':
        DDRH &= (offMask << bits[pin]);
        DDRH |= (onMask << bits[pin]);
       
      break;

      case 'J':
        DDRJ &= (offMask << bits[pin]);
        DDRJ |= (onMask << bits[pin]);
      break;

      case 'L':
        DDRL &= (offMask << bits[pin]);
        DDRL |= (onMask << bits[pin]);
      break;

      default:
        serialDisplay("Error in pinSet function");
      break;
  }

  //No pullups, so clear PORT registers
  PORTA=0b00000000;
  PORTB=0b00000000;
  PORTC=0b00000000;
  PORTD=0b00000000;
  PORTE=0b00000000;
  PORTG=0b00000000;
  PORTH=0b00000000;
  PORTJ=0b00000000;
  PORTL=0b00000000;
}

void adc_init()
{
  // setup the A register
  ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *ADC_DATA;
}
