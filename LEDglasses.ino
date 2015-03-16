#define BUTTON 2
#define RED 6
#define GREEN 5
#define ENC_A 15
#define ENC_B 14
#define ENC_PORT PINC

#define MAX_FREQ 63  // the maximum frequency to allow
#define BOUNCE_DURATION 50   // define an appropriate bounce time in ms for your switches

// the different settings used by the rotary encoder
typedef enum { FREQUENCY, HUE, BRIGHTNESS, NUM_MODES } RotaryMode;

unsigned int period = 1000; // in ms
unsigned int rgb[3] = { 255, 0, 0}; // 0 -255

RotaryMode currentMode = FREQUENCY;
int rotaryCounters[NUM_MODES];

unsigned long lastUpdateTime = 0;
unsigned int phase = 0;

void setup() {
  rotaryCounters[FREQUENCY] = 1;
  rotaryCounters[HUE] = 0;
  rotaryCounters[BRIGHTNESS] = 75;

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

  boolean buttonPressed = wasButtonJustPressed();
  if (buttonPressed) {
    Serial.println("Button pressed");
    currentMode = (RotaryMode) ((currentMode + 1) % NUM_MODES);
  }
  int* currCounter = &rotaryCounters[currentMode];
  
  int8_t encoderChg = read_encoder();
  if (encoderChg == 0 && !buttonPressed) return;
  
  switch (currentMode) {
    case FREQUENCY:
      *currCounter = constrain(*currCounter+encoderChg, 4, 255);
      Serial.print("Freq counter: "); Serial.println(*currCounter);
      analogWrite(GREEN, gamma(*currCounter));    
      analogWrite(RED, 0);
      
      period = 1000UL * 255 / *currCounter / MAX_FREQ;
      Serial.print("period: "); Serial.println(period, DEC);
      break;
    
    case HUE:
      *currCounter = constrain(*currCounter+encoderChg, 0, 255);
      Serial.print("Hue1 counter: "); Serial.println(*currCounter);
      analogWrite(RED, gamma(*currCounter));
      analogWrite(GREEN, 0);
      
      getRGB(rotaryCounters[HUE], 255, rotaryCounters[BRIGHTNESS], rgb);  
      break;
    
    case BRIGHTNESS:
      *currCounter = constrain(*currCounter+encoderChg, 0, 255);
      Serial.print("Brightness counter: "); Serial.println(*currCounter);
      analogWrite(RED, gamma(*currCounter));
      analogWrite(GREEN, 0);

      getRGB(rotaryCounters[HUE], 255, rotaryCounters[BRIGHTNESS], rgb);  
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
  unsigned int rgbEyes[3][2];
  
  for (int eye=0; eye<2; eye++) {    
    for (int c=0; c<3; c++) {
      if (phase < period / 2) {
        rgbEyes[c][eye] = rgb[c];
      } else {
        rgbEyes[c][eye] = 0;
      }
    }
  }

  writeColor(rgbEyes[0][0], rgbEyes[1][0], rgbEyes[2][0], rgbEyes[0][1], rgbEyes[1][1], rgbEyes[2][1]);
}

void writeColor(int Rl, int Gl, int Bl, int Rr, int Gr, int Br) {
 SB_WriteColor(1, Rl, Gl*4, Bl*4);
 SB_WriteColor(0, Rr, Gr*4, Br*4);
 SB_UpdateLEDs();
}


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

