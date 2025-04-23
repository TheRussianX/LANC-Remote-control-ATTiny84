// 8MHz with prescaler 1/8
#include <avr/io.h>
#include <avr/sleep.h>

const int lancIn = 1;   // LANC In on PA1
const int lancOut = 0;  //LANC out on PA0

const int Backlight = 2; //PA2
const int PowerOff = 3;
const int ZoomSpeedH = 4;
const int ZoomSpeedL = 5;
const int ZoomOut = 6;
const int ZoomIn = 7; // PA7

const int Rec = 8; //PB2
const int Mode = 9; //PB1
const int StatusLED = 10;

volatile int top = 101;    // approx 104 microsec to overflow
volatile int zoomspeed = 2; // similar to IR
volatile int mode = 0;

ISR(PCINT1_vect) {  // for PB pins
  int in = PINB;
  if (((in & (1 << PINB2)) == 0) && (mode == 0)) sendLanc(0x18, 0x33);  //Rec
  else if (((in & (1 << PINB2)) == 0) && (mode == 1)) sendLanc(0x18, 0x34);  //Play      
  
  else if ((in & (1 << PINB1)) == 0) { //Mode
    mode++;
    if (mode>1) {
      mode=0;
    }
    blinkLED(1500,0);
  } 
  
}

ISR(PCINT0_vect) {  //for PA pins
  int ia = PINA;
  if (((ia & (1 << PINA2)) == 0) && (mode == 0)) sendLanc(0x28, 0x51); //Backlight
  else if (((ia & (1 << PINA2)) == 0) && (mode == 1)) sendLanc(0x18, 0x38); //Fast Forward
  
  else if ((ia & (1 << PINA3)) == 0) sendLanc(0x18, 0x5E); //Power Off
  
  else if ((ia & (1 << PINA4)) == 0) { //Zoom Speed Increase
    zoomspeed++;
    if (zoomspeed>7){
      zoomspeed = 0;
    }
    blinkLED(1500,0);
  }


  else if ((ia & (1 << PINA5)) == 0) { //Zoom Speed Decrease
    zoomspeed--;
    if (zoomspeed<0){
      zoomspeed = 7;
    }
    blinkLED(1500,0);
  }

  else if (((ia & (1 << PINA6)) == 0) && (mode == 0) && (zoomspeed == 0)) {
    while (digitalRead(6) == LOW) sendLanc(0x28, 0x10); //Zoom Out slowest, sending while the button is pressed
  }
  else if (((ia & (1 << PINA6)) == 0) && (mode == 0) && (zoomspeed == 1)) {
    while (digitalRead(6) == LOW) sendLanc(0x28, 0x12); //Zoom Out
  }
  else if (((ia & (1 << PINA6)) == 0) && (mode == 0) && (zoomspeed == 2)) {
    while (digitalRead(6) == LOW) sendLanc(0x28, 0x14); //Zoom Out
  }
  else if (((ia & (1 << PINA6)) == 0) && (mode == 0) && (zoomspeed == 3)) {
    while (digitalRead(6) == LOW) sendLanc(0x28, 0x16); //Zoom Out
  }
  else if (((ia & (1 << PINA6)) == 0) && (mode == 0) && (zoomspeed == 4)) {
    while (digitalRead(6) == LOW) sendLanc(0x28, 0x18); //Zoom Out
  }
  else if (((ia & (1 << PINA6)) == 0) && (mode == 0) && (zoomspeed == 5)) {
    while (digitalRead(6) == LOW) sendLanc(0x28, 0x1A); //Zoom Out
  }
  else if (((ia & (1 << PINA6)) == 0) && (mode == 0) && (zoomspeed == 6)) {
    while (digitalRead(6) == LOW) sendLanc(0x28, 0x1C); //Zoom Out
  }
  else if (((ia & (1 << PINA6)) == 0) && (mode == 0) && (zoomspeed == 7)) {
    while (digitalRead(6) == LOW) sendLanc(0x28, 0x1E); //Zoom Out fastest, sending while the button is pressed
  } 
  else if (((ia & (1 << PINA6)) == 0) && (mode == 1)) sendLanc(0x18, 0x30); //Stop

 
  else if (((ia & (1 << PINA7)) == 0) && (mode == 0) && (zoomspeed == 0)){
    while (digitalRead(7) == LOW) sendLanc(0x28, 0x00); //Zoom In slowest, sending while the button is pressed
  }
  else if (((ia & (1 << PINA7)) == 0) && (mode == 0) && (zoomspeed == 1)){
    while (digitalRead(7) == LOW) sendLanc(0x28, 0x02); //Zoom In
  }
  else if (((ia & (1 << PINA7)) == 0) && (mode == 0) && (zoomspeed == 2)){
    while (digitalRead(7) == LOW) sendLanc(0x28, 0x04); //Zoom In
  }
  else if (((ia & (1 << PINA7)) == 0) && (mode == 0) && (zoomspeed == 3)){
    while (digitalRead(7) == LOW) sendLanc(0x28, 0x06); //Zoom In
  }
  else if (((ia & (1 << PINA7)) == 0) && (mode == 0) && (zoomspeed == 4)){
    while (digitalRead(7) == LOW) sendLanc(0x28, 0x08); //Zoom In
  }
  else if (((ia & (1 << PINA7)) == 0) && (mode == 0) && (zoomspeed == 5)){
    while (digitalRead(7) == LOW) sendLanc(0x28, 0x0A); //Zoom In
  }
  else if (((ia & (1 << PINA7)) == 0) && (mode == 0) && (zoomspeed == 6)){
    while (digitalRead(7) == LOW) sendLanc(0x28, 0x0C); //Zoom In
  }
  else if (((ia & (1 << PINA7)) == 0) && (mode == 0) && (zoomspeed == 7)){
    while (digitalRead(7) == LOW) sendLanc(0x28, 0x0E); //Zoom In fastest, sending while the button is pressed
  }
  else if (((ia & (1 << PINA7)) == 0) && (mode == 1)) sendLanc(0x18, 0x36); //Rewind

}

void SetupTimer() {
  // CTC timer no output TOP value OCR1A
  TCCR1B = (1 << WGM12) | (1 << CS11); // prescaler 1/8
  OCR1A = top;
}

void delayTimer(int top){  // used to generate variable delays
  TIFR1 = (1<<OCF1A); //clear OCF1A flag
  TCNT1=0; // start counting from 0
  while ((TIFR1&(1<<OCF1A)) == 0); //does nothing until OCF1A flag is raised
}

void sendByte(unsigned long command){ //uswed to send a byte
  for (int Bit = 0; Bit<8; Bit++) {
    if (command & (unsigned long) 1<<Bit) {
      PORTA |= (1<<PORTA0); //set PA0 to HIGH
      delayTimer(101);
      } else {
        PORTA &= ~(1<<PORTA0); //set PA0 to LOW
        delayTimer(101);
        }
  }
}

void sendLanc(unsigned long raw_byte0, unsigned long raw_byte1){
  int cmdRepeatCount = 0;
  
  while (cmdRepeatCount < 5) { //repeat 5 times to make sure the camera accepts the command
    while (pulseIn(lancIn, HIGH) < 5000) {
      //"pulseIn, HIGH" catches any 0V TO +5V TRANSITION and waits until the LANC line goes back to 0V 
      //"pulseIn" also returns the pulse duration so we can check if the previous +5V duration was long enough (>5ms) to be the pause before a new 8 byte data packet
      //Loop till pulse duration is >5ms
    }
    //LOW after long pause means the START bit of Byte 0 is here
    delayTimer(101); //wait START bit duration
    sendByte(raw_byte0);
    //Byte 0 is written now put LANC line back to +5V
    //digitalWrite(cmdPin, LOW);
    PORTA &= ~(1<<PORTA0); //set PA0 to LOW
    delayTimer(9); //make sure to be in the stop bit before byte 1

    while (digitalRead(lancIn)) {
      //Loop as long as the LANC line is +5V during the stop bit
    }
    //0V after the previous stop bit means the START bit of Byte 1 is here
    delayTimer(101);  //wait START bit duration
    sendByte(raw_byte1);
    //Byte 1 is written now put LANC line back to +5V
    //digitalWrite(cmdPin, LOW);
    PORTA &= ~(1<<PORTA0); //set PA0 to LOW
    delayTimer(9); //make sure to be in the stop bit before byte 2

    cmdRepeatCount++;  //increase repeat count by 1
    /*Control bytes 0 and 1 are written, now don’t care what happens in Bytes 2 to 7
    and just wait for the next start bit after a long pause to send the first two command bytes again.*/
  }
}

void blinkLED(int on, int off){
  PORTB |= (1<<PORTB0); //turn LED on
  for (int count = 0; count<on; count++) {
    delayTimer(101);
    }
  PORTB &= ~(1<<PORTB0); // turn LED off
  for (int count = 0; count<off; count++) {
      delayTimer(101);
    }
}

void setup() {
  OSCCAL=119; // 8MHz MCU, change to your calibrated MCU value
  pinMode(lancOut, OUTPUT);
  pinMode(lancIn, INPUT);
  pinMode(Backlight, INPUT_PULLUP);
  pinMode(PowerOff, INPUT_PULLUP);
  pinMode(ZoomSpeedH, INPUT_PULLUP);
  pinMode(Mode, INPUT_PULLUP);
  pinMode(ZoomOut, INPUT_PULLUP);
  pinMode(ZoomIn, INPUT_PULLUP);
  pinMode(Rec, INPUT_PULLUP);
  pinMode(ZoomSpeedL, INPUT_PULLUP);
  pinMode(PB3, INPUT_PULLUP); //Reset must not be left floating
  pinMode(StatusLED, OUTPUT);
  PORTA &= ~(1<<PORTA0); //PA0 LOW for sendLanc() to work through LANC interface to camera
  SetupTimer();

    // Configure pin change interrupts to wake on button presses
  PCMSK1 |= (1<< PCINT9) | (1<< PCINT10); //Mask PB1 and PB2
  PCMSK0 |= (1<< PCINT2) | (1<< PCINT3) | (1<< PCINT4) | (1<< PCINT5) | (1<< PCINT6)| (1<< PCINT7); //Mask PA pins from 2 to 7
  GIMSK |= (1 << PCIE1) | (1 << PCIE0);  // Enable interrupts
  GIFR |= (1 << PCIF1) | (1 << PCIF0);   // Clear interrupt flag
  // Disable what we don't need to save power
  ADCSRA &= ~(1 << ADEN);         // Disable ADC
  PRR = 1 << PRUSI | 1 << PRADC;  // Turn off clocks to unused peripherals
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}

void loop() {
  sleep_enable();
  sleep_cpu();
}