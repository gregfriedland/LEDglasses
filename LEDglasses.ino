#define BUTTON 2
#define RED 6
#define GREEN 5
#define ENC_A 15
#define ENC_B 14
#define ENC_PORT PINC

#define MAX_FREQ 63  // the maximum frequency to allow
#define BOUNCE_DURATION 50   // define an appropriate bounce time in ms for your switches

#define SQUARE 0
#define BLANK1 1
#define SINE 2
#define BLANK2 3
#define TRIANGLE 4
#define NUM_WAVE_SHAPES 5

// the different settings used by the rotary encoder
#define FREQUENCY 0
#define WAVE_SHAPE 1
#define HUE1 2
#define HUE2 3
#define PHASE_OFFSET 4
#define NUM_SETTINGS 5

unsigned int period = 1000; // in ms
unsigned int rightPhaseOffset = 0; // 0 - 1023 -> 0 -> 2PI
unsigned int rgb1[3] = { 255, 0, 0}; // 0 -255
unsigned int rgb2[3] = { 255, 0, 0};
unsigned short waveShape = SQUARE;

//unsigned long lastSettingsUpdate = 0;
unsigned long lastUpdateTime = 0;
unsigned int phase = 0;

void setup() {
  /* Setup encoder pins as inputs */
  pinMode(ENC_A, INPUT);
  digitalWrite(ENC_A, HIGH);
  pinMode(ENC_B, INPUT);
  digitalWrite(ENC_B, HIGH);
  
  pinMode(BUTTON, INPUT);
  digitalWrite(BUTTON, HIGH);

  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
    
  SB_init();
  randomSeed(analogRead(18));
  
  Serial.begin (115200);
  Serial.println("Start");
}
 
void loop() {
  draw();
  delay(1);
  updateSettings();
}

void updateSettings() {
  static int currentSetting = 0;
  static int freqCounter = 0;
  static int waveShapeCounter = 0;
  static int hue1Counter = 0;
  static int hue2Counter = 0;
  static int phaseOffsetCounter = 0;

  boolean buttonPressed = wasButtonJustPressed();
  if (buttonPressed) {
    Serial.println("Button pressed");
    currentSetting = (currentSetting + 1) % NUM_SETTINGS;
  }
  
  int8_t encoderChg = read_encoder();
  if (encoderChg == 0 && !buttonPressed) return;
  
  switch (currentSetting) {
    case FREQUENCY:
      freqCounter = constrain(freqCounter+encoderChg, 4, 255);
      Serial.print("Freq counter: "); Serial.println(freqCounter);
      analogWrite(GREEN, gamma(freqCounter));    
      analogWrite(RED, 0);
      
      // two periods actually to allow alternating colors
      period = 1000UL * 255 * 2 / freqCounter / MAX_FREQ;
      Serial.print("period: "); Serial.println(period, DEC);
      break;
    
    case WAVE_SHAPE:
      waveShapeCounter = constrain(waveShapeCounter+encoderChg, 0, 255);
      Serial.print("Wave shape counter: "); Serial.println(waveShapeCounter);
      analogWrite(RED, gamma(waveShapeCounter));
      analogWrite(GREEN, 0);
      
      waveShape = waveShapeCounter * NUM_WAVE_SHAPES / 256;
      break;
    
    case HUE1:
      hue1Counter = constrain(hue1Counter+encoderChg, 0, 255);
      Serial.print("Hue1 counter: "); Serial.println(hue1Counter);
      analogWrite(RED, gamma(hue1Counter));
      analogWrite(GREEN, 0);
      
      getRGB(hue1Counter, 255, 255, rgb1);  
      break;
    
    case HUE2:
      hue2Counter = constrain(hue2Counter+encoderChg, 0, 255);
      Serial.print("Hue2 counter: "); Serial.println(hue2Counter);
      analogWrite(RED, gamma(hue2Counter));
      analogWrite(GREEN, 0);

      getRGB(hue2Counter, 255, 255, rgb2);  
      break;
    
    case PHASE_OFFSET:
      phaseOffsetCounter = constrain(phaseOffsetCounter+encoderChg, 0, 255);
      Serial.print("Phase offset counter: "); Serial.println(phaseOffsetCounter);
      analogWrite(RED, gamma(phaseOffsetCounter));
      analogWrite(GREEN, 0);
      
      rightPhaseOffset = phaseOffsetCounter;
      break;
  }
}
  

/* returns change in encoder state (-1,0,1) */
int8_t read_encoder() {
  static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t old_AB = 0;
  /**/
  old_AB <<= 2;                   //remember previous state
  old_AB |= ( ENC_PORT & 0x03 );  //add current state
  return -( enc_states[( old_AB & 0x0f )]);
}

void draw() {
  phase = (phase +  millis() - lastUpdateTime) % period;
  lastUpdateTime = millis();
  unsigned int phases[2];
  phases[0] = phase;
  phases[1] = (phase + long(rightPhaseOffset) * period / 255) % period;
  Serial.print(period); Serial.print(" "); Serial.print(phases[0]); Serial.print(" "); Serial.println(phases[1]);
  unsigned int rgbEyes[3][2];
  
  switch(waveShape) {
    case SINE:  
      for (int eye=0; eye<2; eye++) {    
        unsigned long sinePhase = 2 * 255UL * phases[eye] / period;
        int sineVal = sine(sinePhase % 255);
        if (sinePhase <= 255 * 3 / 4 || sinePhase > 255 * 7 / 4) {
          for (int c=0; c<3; c++) {
            rgbEyes[c][eye] = rgb1[c] * sineVal / 255;
          }
        } else {
          for (int c=0; c<3; c++) {
            rgbEyes[c][eye] = rgb2[c] * sineVal / 255;
          }
        }
      }
      break;
    
    case SQUARE:
    case TRIANGLE:
      for (int eye=0; eye<2; eye++) {    
        for (int c=0; c<3; c++) {
          if (phases[eye] < period / 4) {
            rgbEyes[c][eye] = rgb1[c];
          } else if (phases[eye] < period / 2) {
            rgbEyes[c][eye] = 0;
          } else if (phases[eye] < period * 3 / 4) {
            rgbEyes[c][eye] = rgb2[c];
          } else {
            rgbEyes[c][eye] = 0;
          }
        }
      }
      break;
    
    case BLANK1:
    case BLANK2:
      for (int eye=0; eye<2; eye++) {    
        for (int c=0; c<3; c++) {
          rgbEyes[c][eye] = 0;
        }
      }
      break;      
  }
  writeColor(rgbEyes[0][0], rgbEyes[1][0], rgbEyes[2][0], rgbEyes[0][1], rgbEyes[1][1], rgbEyes[2][1]);
}

void writeColor(int Rl, int Gl, int Bl, int Rr, int Gr, int Br) {
 SB_WriteColor(1, Rl, Gl*4, Bl*4);
 SB_WriteColor(0, Rr, Gr*4, Br*4);
 SB_UpdateLEDs();
}

//int buttonState = HIGH;             // the current reading from the input pin
//int lastButtonState = LOW;   // the previous reading from the input pin
//long lastDebounceTime = 0;  // the last time the output pin was toggled
//long debounceDelay = 50;    // the debounce time; increase if the output flickers

boolean wasButtonJustPressed() {
  static int buttonState = HIGH;
  static int lastReading = LOW;
  static long lastDebounceTime = 0;

  int reading = digitalRead(BUTTON);

  // If the switch changed, due to noise or pressing:
  if (reading != lastReading) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  } 
  lastReading = reading;

  if ((millis() - lastDebounceTime) > BOUNCE_DURATION && lastDebounceTime != 0) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    boolean pressed = (buttonState == HIGH && buttonState != reading);
    buttonState = reading;
    return pressed;
  } else {
    return false;
  }
}

