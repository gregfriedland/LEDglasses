#define BUTTON 2
#define ENC_RED 5
#define ENC_GREEN 6
#define ENC_A 15
#define ENC_B 14
#define ENC_PORT PINC

#define MIN_FREQ 1
#define MAX_FREQ 63  // the maximum frequency to allow
#define MAX_HUE 360
#define MAX_SAT 255
#define MAX_BRIGHTNESS 255
#define BOUNCE_DURATION 50   // define an appropriate bounce time in ms for your switches
#define MAX_ENC_VAL 1023

#define p(...) Serial.print(__VA_ARGS__)


// the different settings used by the rotary encoder
typedef enum { FREQUENCY, HUE, BRIGHTNESS, GUIDED, NUM_MODES } RotaryMode;

typedef struct GuidedState {
  uint32_t precision;   // precision to store freq & hue at
  uint16_t dChangeOdds; // change direction in freq or hue with 1:dChangeOdds prob
  uint32_t freq;
  uint32_t hue;
  int32_t dfreq;
  int32_t dhue;
};


GuidedState guidedState;
unsigned long period = 1000000UL; // in us
unsigned int rgb[3] = { 0, 0, 0};

RotaryMode currMode = FREQUENCY;
int rotaryCounters[NUM_MODES];

unsigned long lastUpdateTime = 0;
unsigned long phase = 0;

void setup() {
  rotaryCounters[FREQUENCY] = 0;
  rotaryCounters[HUE] = 0;
  rotaryCounters[BRIGHTNESS] = 75;
  rotaryCounters[GUIDED] = 10;

  guidedState.dChangeOdds = 100;
  guidedState.precision = 1000;
  guidedState.freq = 1 * guidedState.precision;
  guidedState.hue = 0;
  guidedState.dfreq = guidedState.precision / 100;
  guidedState.dhue = guidedState.precision / 10;

  /* Setup encoder pins as inputs */
  pinMode(ENC_A, INPUT);
  digitalWrite(ENC_A, HIGH);
  pinMode(ENC_B, INPUT);
  digitalWrite(ENC_B, HIGH);
  
  pinMode(BUTTON, INPUT);
  digitalWrite(BUTTON, HIGH);

  pinMode(ENC_RED, OUTPUT);
  pinMode(ENC_GREEN, OUTPUT);
  analogWrite(ENC_RED, 10);
    
  SB_init();
  randomSeed(analogRead(18));
  
  Serial.begin(115200);
  p("Start\n");

  getRGB(rotaryCounters[HUE], MAX_SAT, rotaryCounters[BRIGHTNESS], rgb);    
}
 
void loop() {
  draw();
  delay(5);

  updateSettings();
  if (currMode == GUIDED)
    updateGuided();
}

void updateSettings() {
  boolean buttonPressed = wasButtonJustPressed();
  if (buttonPressed) {
    p("Button pressed\n");
    currMode = (RotaryMode) ((currMode + 1) % NUM_MODES);
  }
  int* currCounter = &rotaryCounters[currMode];
  
  int8_t encoderChg = read_encoder();
  if (encoderChg == 0 && !buttonPressed) return;

  switch (currMode) {
    case FREQUENCY:
      *currCounter = constrain(*currCounter+encoderChg, 0, MAX_ENC_VAL);
      period = 1000000UL / MAX_FREQ * MAX_ENC_VAL / *currCounter;
      period = constrain(period, 1000000UL / MAX_FREQ, 1000000UL/MIN_FREQ);
      p("Freq counter: "); p(*currCounter); p("\n");
      p("Freq: "); p(1000000UL/period, DEC); p("\n");
      break;
    
    case HUE:
      *currCounter = (*currCounter+encoderChg+MAX_HUE) % MAX_HUE;
      p("Hue1 counter: "); p(*currCounter); p("\n");
      break;
    
    case BRIGHTNESS:
      *currCounter = constrain(*currCounter+encoderChg, 0, MAX_BRIGHTNESS);
      p("Brightness counter: "); p(*currCounter); p("\n");
      break;
  }

  analogWrite(ENC_RED, currMode == FREQUENCY ? max(*currCounter/4, 10) : 0);
  analogWrite(ENC_GREEN, currMode == HUE || currMode == BRIGHTNESS ? max(*currCounter, 10) : 0);
  
  getRGB(rotaryCounters[HUE], MAX_SAT, rotaryCounters[BRIGHTNESS], rgb);  
}
  

void updateGuided()
{
  int speedMult = rotaryCounters[currMode];

  if ((guidedState.freq / guidedState.precision <= MIN_FREQ && guidedState.dfreq < 0) ||
      (guidedState.freq / guidedState.precision >= MAX_FREQ && guidedState.dfreq > 0))
    guidedState.dfreq *= -1;
  else if (random(guidedState.dChangeOdds) == 0)
    guidedState.dfreq *= -1;
  
  if (random(guidedState.dChangeOdds) == 0)
    guidedState.dhue *= -1;

  guidedState.freq += guidedState.dfreq * speedMult;
  guidedState.freq = constrain(guidedState.freq, MIN_FREQ*guidedState.precision, MAX_FREQ*guidedState.precision);
  period = 1000000UL / (guidedState.freq / guidedState.precision);

  guidedState.hue += guidedState.dhue * speedMult;
  guidedState.hue = (guidedState.hue + MAX_HUE*guidedState.precision) % (MAX_HUE*guidedState.precision);
  getRGB(guidedState.hue/guidedState.precision, MAX_SAT, rotaryCounters[BRIGHTNESS], rgb);  

  // p("Guided: freq="); p(guidedState.freq/guidedState.precision); 
  // p("; hue="); p(guidedState.hue/guidedState.precision); p("\n");
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
  phase = (phase +  micros() - lastUpdateTime) % period;
  lastUpdateTime = micros();
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

