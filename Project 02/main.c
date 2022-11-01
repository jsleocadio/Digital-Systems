// C++ code
//
// Set 'TOP' for PWM resolution.  Assumes 16 MHz clock.
// TOP = (Mclock / N*fpwm) - 1
// Mclock = 16*10^6 or 16 MHz | N = pre-scale = 64 | fpwm = 50 Hz
const unsigned int TOP = 4999;
const unsigned int TMINUS90 = 249; // TMINUS90 is the value where the servo motor turns -90º, to the left.
const unsigned int T90 = 499; // T90 is the value where the servo motor turns 90º, to the right.
int ledRight = LOW;
int ledLeft = LOW;
int headLights = LOW;
int wiper = LOW;
int ignitionState = 0;
int fuel = 0;
int speed = 0;
unsigned long start = 0;

//Set Ports to be used
void DDRBegin()
{
  DDRB |= 0b00000010;	//Set PORTB2 as OUTPUT
  PORTB &= 0;
  DDRD |= 0b11110000;	//Set PORTD4|PORTD5|PORTD6|PORTD7 as OUTPUT
  PORTD &= 0;
}

//Set Analogic inputs and ADC
void ADCBegin()
{
  ADMUX |= (1<<REFS0);
  ADCSRA |= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
}

void PWMBegin()
{
  // Stop Timer/Counter1
  TCCR1A = 0;  // Timer/Counter1 Control Register A
  TCCR1B = 0;  // Timer/Counter1 Control Register B
  
  ICR1 = TOP;
  
  // Set clock prescale to 64
  TCCR1B |= (1 << CS11) | (1 << CS10); //pre-scaler 64

  // Set to Timer/Counter1 to Waveform Generation Mode 14: Fast PWM with TOP set by ICR1
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12) ;
}

void PWMEnable1A()
{
  // Enable Fast PWM on Pin 9
  TCCR1A |= (1 << COM1A1);
}

//Set Right LED to blink for 0.5s
void blinkRightLed(){
  if (ledRight){
    PORTD |= 1<<4;
    _delay_ms(500);
    PORTD &= ~(1<<4);
    _delay_ms(450);
  }
}

//Set Left LED to blink for 0.5s
void blinkLeftLed(){
  if (ledLeft){
    PORTD |= 1<<5;
    _delay_ms(500);
    PORTD &= ~(1<<5);
    _delay_ms(450);
  }
}

//Turn on the Eletronic Injection LED
void turnOnEletronicInjection(){
  PORTD |= 1<<6;
}

//Turn on the Headlights
void turnOnHeadLights(){
  if (headLights) {
    PORTD |= 1<<7;
  }
}

//Calculate voltage on ignition
void getVoltage(){
  ADMUX &= 0b11110000;
  ADMUX |= 0b00000000;
  ADCSRA |= (1<<ADSC);
  while(!(ADCSRA & (1<<ADIF)));
  ignitionState = ADC;
}

//Print fuel gauge
void printFuel(){
  ADMUX &= 0b11110000;
  ADMUX |= 0b00000001;
  ADCSRA |= (1<<ADSC);
  while(!(ADCSRA & (1<<ADIF)));
  fuel = float(ADC)/1023 * 100;
  Serial.print("Fuel: ");
  Serial.print(fuel);
  Serial.println(" %.");
}

//Get Vehicle Speed
void printSpeed(){
  ADMUX &= 0b11110000;
  ADMUX |= 0b00000010;
  ADCSRA |= (1<<ADSC);
  while(!(ADCSRA & (1<<ADIF)));
  speed = float(ADC)/1023 * 120;
  Serial.print("Speedmeter: ");
  Serial.print(speed);
  Serial.println(" Km/h.");
}

void PWM1A(unsigned int PWMValue)
{
  OCR1A = PWMValue;
}

void turnOnWiper(){
  if (wiper){
    PWM1A(T90);
    _delay_ms(600);
    PWM1A(TMINUS90);
    _delay_ms(500);
  }
}

//Set Ports, Pins and Registers:
void setup(){
  Serial.begin(9600);
  DDRBegin();			//Call Ports
  ADCBegin();			//Call ADC
  PWMBegin();			//Call PWM
  
  PWM1A(0);
  PWMEnable1A();
}


void loop(){
   //Make sure that all Outputs are LOW
    PORTD &= 0;
    
    //Get Voltage
    getVoltage();
    
    //Verify pressed buttons:
    if (PINB & 1<<2){
      ledRight = !ledRight;
    }
    if (PINB & 1<<3){
      ledLeft = !ledLeft;
    }
    if (PINB & 1<<4){
      headLights = !headLights;
    }
    if (PINB & 1<<5){
      wiper = !wiper;
    }
    //Last Stage of ignition: Can turn on everything
    if (ignitionState == 682){
      turnOnEletronicInjection();
      blinkRightLed();
      blinkLeftLed();
      turnOnHeadLights();
      turnOnWiper();
      unsigned long end = millis();
      if (end - start >= 500) {
        start = end;
        printFuel();
        printSpeed();
      }
    } 
    // Second stage of ignition: Can turn something
    else if (ignitionState >= 341){
      blinkRightLed();
      blinkLeftLed();
      turnOnHeadLights();
      unsigned long end = millis();
      if (end - start >= 500) {
        start = end;
        printFuel();
      }
    }
  _delay_ms(50);
}
  
 
/*
ADCSRA: ADC Control Status Register A
7 (ADEN) - Enable AD conversion
6 (ADSC) - Start ADC conversion
5 (ADATE)- Enable auto trigger
4 (ADIF) - ADC Interrupt Flag
3 (ADIE)- Enable ADC Interrupt
2:0 (ADPS2) - ADC prescale

ADMUX: ADC Multiplexer Selection Register
7:6 (REFSx) - Voltage Reference
5 (ADLAR) - ADC Left Adjust
4 - Reserver
3:0 - Analog pin mux
*/