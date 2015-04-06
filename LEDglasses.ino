#include "FastLED.h"

// the different settings used by the rotary encoder
typedef enum { BRIGHTNESS, GUIDED, FREQUENCY, HUE, NUM_MODES } RotaryMode;


#define ENC_BUTTON 4
#define ENC_A A1
#define ENC_B A0
#define ENC_PORT PINC
#define ENC_INCR 3 // how fast to adjust from one encoder change
#define LED_PIN 2

#define DEFAULT_BRIGHTNESS 150
#define DEFAULT_GUIDED_SPEED 3
#define DEFAULT_MODE BRIGHTNESS

#define MIN_FREQ 6
#define MAX_FREQ 45  // the maximum frequency to allow
#define MAX_HUE 255
#define MAX_SAT 255
#define MAX_BRIGHTNESS 255
#define MAX_GUIDED_SPEED_MULT 100
#define BOUNCE_DURATION 50   // define an appropriate bounce time in ms for your switches
#define MAX_ENC_VAL 1023

#define NUM_LEDS 8 // There are 8 leds total although we use only the 3 on either end
const bool use_leds[] = {1,1,1,0,0,1,1,1};

#define p(...) Serial.print(__VA_ARGS__)

CRGB leds[NUM_LEDS];


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
CRGB rgb;

RotaryMode currMode = DEFAULT_MODE;
int rotaryCounters[NUM_MODES];

unsigned long lastUpdateTime = 0;
unsigned long phase = 0;

void setup() {
  rotaryCounters[FREQUENCY] = MIN_FREQ * MAX_ENC_VAL / MAX_FREQ;
  rotaryCounters[HUE] = 0;
  rotaryCounters[BRIGHTNESS] = DEFAULT_BRIGHTNESS;
  rotaryCounters[GUIDED] = DEFAULT_GUIDED_SPEED;

  guidedState.dChangeOdds = 250;
  guidedState.precision = 200;
  guidedState.freq = 1 * guidedState.precision;
  guidedState.hue = 0;
  guidedState.dfreq = (guidedState.precision * MAX_FREQ) / 10000;
  guidedState.dhue = (guidedState.precision * MAX_HUE) / 10000;

  init_encoder();

  pinMode(ENC_BUTTON, INPUT);
  digitalWrite(ENC_BUTTON, HIGH);

  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.setBrightness( 255 ); // brightness is controlled elsewhere

  randomSeed(analogRead(18));
  
  period = 1000000UL / MAX_FREQ * MAX_ENC_VAL / rotaryCounters[FREQUENCY];

  hsv2rgb_rainbow(CHSV(rotaryCounters[HUE], MAX_SAT, rotaryCounters[BRIGHTNESS]), rgb);

  Serial.begin(115200);
  p("Start\n");
}
 
void loop() {
  draw();
  delay(1);

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

    case GUIDED:
      *currCounter = constrain(*currCounter+encoderChg, 0, MAX_GUIDED_SPEED_MULT);
      p("Guided speed counter: "); p(*currCounter); p("\n");
      break;      
  }

  hsv2rgb_rainbow(CHSV(rotaryCounters[HUE], MAX_SAT, rotaryCounters[BRIGHTNESS]), rgb);
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
  hsv2rgb_rainbow(CHSV(guidedState.hue/guidedState.precision, MAX_SAT, rotaryCounters[BRIGHTNESS]), rgb);


  // p("Guided: freq="); p(guidedState.freq/guidedState.precision); 
  // p("; hue="); p(guidedState.hue/guidedState.precision); p("\n");
}


static uint8_t enc_prev_pos   = 0;
static uint8_t enc_flags      = 0;

void init_encoder() {
  /* Setup encoder pins as inputs */
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  // get an initial reading on the encoder pins
  if (digitalRead(ENC_A) == LOW) {
    enc_prev_pos |= (1 << 0);
  }
  if (digitalRead(ENC_B) == LOW) {
    enc_prev_pos |= (1 << 1);
  }
}
/* returns change in encoder state (-1,0,1) */
// From Adafruit: https://learn.adafruit.com/pro-trinket-rotary-encoder
int8_t read_encoder() {

  int8_t enc_action = 0; // 1 or -1 if moved, sign is direction
 
  // note: for better performance, the code will use
  // direct port access techniques
  // http://www.arduino.cc/en/Reference/PortManipulation
  uint8_t enc_cur_pos = 0;
  // read in the encoder state first
  if ((ENC_PORT & digitalPinToBitMask(ENC_A)) == 0) {
    enc_cur_pos |= (1 << 0);
  }
  if ((ENC_PORT & digitalPinToBitMask(ENC_B)) == 0) {
    enc_cur_pos |= (1 << 1);
  }
 
  // if any rotation at all
  if (enc_cur_pos != enc_prev_pos)
  {
    if (enc_prev_pos == 0x00)
    {
      // this is the first edge
      if (enc_cur_pos == 0x01) {
        enc_flags |= (1 << 0);
      }
      else if (enc_cur_pos == 0x02) {
        enc_flags |= (1 << 1);
      }
    }
 
    if (enc_cur_pos == 0x03)
    {
      // this is when the encoder is in the middle of a "step"
      enc_flags |= (1 << 4);
    }
    else if (enc_cur_pos == 0x00)
    {
      // this is the final edge
      if (enc_prev_pos == 0x02) {
        enc_flags |= (1 << 2);
      }
      else if (enc_prev_pos == 0x01) {
        enc_flags |= (1 << 3);
      }
 
      // check the first and last edge
      // or maybe one edge is missing, if missing then require the middle state
      // this will reject bounces and false movements
      if (bit_is_set(enc_flags, 0) && (bit_is_set(enc_flags, 2) || bit_is_set(enc_flags, 4))) {
        enc_action = 1;
      }
      else if (bit_is_set(enc_flags, 2) && (bit_is_set(enc_flags, 0) || bit_is_set(enc_flags, 4))) {
        enc_action = 1;
      }
      else if (bit_is_set(enc_flags, 1) && (bit_is_set(enc_flags, 3) || bit_is_set(enc_flags, 4))) {
        enc_action = -1;
      }
      else if (bit_is_set(enc_flags, 3) && (bit_is_set(enc_flags, 1) || bit_is_set(enc_flags, 4))) {
        enc_action = -1;
      }
 
      enc_flags = 0; // reset for next time
    }
  }
 
  enc_prev_pos = enc_cur_pos;
 
  return enc_action * ENC_INCR;
}


void draw() {
  phase = (phase +  micros() - lastUpdateTime) % period;
  lastUpdateTime = micros();
  
  for (int i = 0; i < NUM_LEDS; i++) {
    if (use_leds[i]) {
      for (int c=0; c<3; c++) {
        if (phase < period / 2) {
          leds[i][c] = rgb[c];
        } else {
          leds[i][c] = 0;
        }
      }
    }
  }

  FastLED.show();
}


boolean wasButtonJustPressed() {
  static int buttonState = HIGH;
  static int lastReading = LOW;
  static long lastDebounceTime = 0;

  int reading = digitalRead(ENC_BUTTON);

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

