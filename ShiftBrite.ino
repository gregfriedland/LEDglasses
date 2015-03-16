
// Defines for use with Arduino functions
#define clockpin   13 // CI
#define enablepin  10 // EI
#define latchpin    9 // LI
#define datapin    11 // DI
 
/* Ports and Pins
 
 Direct port access is much faster than digitalWrite.
 You must match the correct port and pin as shown in the table below.
 
 Arduino Pin        Port        Pin
 13 (SCK)           PORTB       5
 12 (MISO)          PORTB       4
 11 (MOSI)          PORTB       3
 10 (SS)            PORTB       2
 9                  PORTB       1
 8                  PORTB       0
 7                  PORTD       7
 6                  PORTD       6
 5                  PORTD       5
 4                  PORTD       4
 3                  PORTD       3
 2                  PORTD       2
 1 (TX)             PORTD       1
 0 (RX)             PORTD       0
 A5 (Analog)        PORTC       5
 A4 (Analog)        PORTC       4
 A3 (Analog)        PORTC       3
 A2 (Analog)        PORTC       2
 A1 (Analog)        PORTC       1
 A0 (Analog)        PORTC       0
 
 */
#define CLKPORT PORTB
#define ENAPORT PORTB
#define LATPORT PORTB
#define DATPORT PORTB
#define CLKPIN  5
#define ENAPIN  2
#define LATPIN  1
#define DATPIN  3
 
// Variables for communication
unsigned long SB_CommandPacket;
int SB_CommandMode;
int SB_BlueCommand;
int SB_RedCommand;
int SB_GreenCommand;
 
// Define number of ShiftBrite modules
#define NumLEDs 2
 
// Create LED value storage array
int LEDChannels[NumLEDs][3] = {0};
 

// Load values into SPI register
void SB_SendPacket() {
    if (SB_CommandMode == B01) {
     SB_RedCommand = 127;
     SB_GreenCommand = 110;
     SB_BlueCommand = 110;
    }
 
    SPDR = SB_CommandMode << 6 | SB_BlueCommand>>4;
    while(!(SPSR & (1<<SPIF)));
    SPDR = SB_BlueCommand<<4 | SB_RedCommand>>6;
    while(!(SPSR & (1<<SPIF)));
    SPDR = SB_RedCommand << 2 | SB_GreenCommand>>8;
    while(!(SPSR & (1<<SPIF)));
    SPDR = SB_GreenCommand;
    while(!(SPSR & (1<<SPIF)));
}
 
// Latch values into PWM registers
void SB_Latch() {
  delayMicroseconds(1);
  LATPORT += (1 << LATPIN);
  //ENAPORT += (1 << ENAPIN);
  delayMicroseconds(1);
  //ENAPORT &= ~(1 << ENAPIN);
  LATPORT &= ~(1 << LATPIN);
 
}
 
void SB_WriteColor(int led, int r, int g, int b) {
  LEDChannels[led][0] = r;
  LEDChannels[led][1] = g;
  LEDChannels[led][2] = b;
  //Serial.print(led); Serial.print(" "); Serial.print(r); Serial.print(" "); Serial.print(g); Serial.print(" "); Serial.println(b);
}

// Send all array values to chain
void SB_UpdateLEDs() {
  SB_CommandMode = B00; // Write to PWM control registers
  for (int i = 0; i<NumLEDs; i++) {
    SB_RedCommand = LEDChannels[i][0] & 1023;
    SB_GreenCommand = LEDChannels[i][1] & 1023;
    SB_BlueCommand = LEDChannels[i][2] & 1023;
    SB_SendPacket();
  }
  SB_Latch();
 
  SB_CommandMode = B01; // Write to current control registers
  for (int z = 0; z < NumLEDs; z++) SB_SendPacket();   
  SB_Latch();
}
 
// Fade between two colors at specified interval
//void fadeall(int rate, int fromred, int fromgreen, int fromblue, int tored, int togreen, int toblue) {
// for (int i = 0; i < 33; i++) {
// 
//     for (int j1 = 0; j1 < NumLEDs; j1++) {
//       LEDChannels[j1][0] = (fromred * (32 - i) + tored * i)/32;
//       LEDChannels[j1][1] = (fromgreen * (32 - i) + togreen * i)/32;
//       LEDChannels[j1][2] = (fromblue * (32 - i) + toblue * i)/32;
//     }
// 
//    WriteLEDArray();
//    delay(rate);
// 
//   }   
//}
 
void SB_init() {
  pinMode(datapin, OUTPUT);
  pinMode(latchpin, OUTPUT);
  pinMode(enablepin, OUTPUT);
  pinMode(clockpin, OUTPUT);
  digitalWrite(latchpin, LOW);
  digitalWrite(enablepin, LOW);
  SPCR = (1<<SPE)|(1<<MSTR)|(0<<SPR1)|(0<<SPR0);
}
