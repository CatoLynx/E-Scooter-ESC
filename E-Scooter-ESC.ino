#include <avr/io.h>
#include "config.h"
#include "notes.h"


typedef struct music {
  uint8_t* notes;
  uint8_t* durations;
  uint16_t tempo;
  uint8_t freqMultiplier;
  uint16_t  length;
} music_t;

typedef struct rollingAvg {
  uint16_t values[10];
  uint16_t nextIndex;
  uint16_t len;
} rollingAvg_t;


const uint16_t freqs_taurus[16] = {
  N_D3,
  N_E3,
  N_F3,
  N_G3,
  N_A3,
  N_B3,
  N_C4,
  N_D4,
  N_E4,
  N_F4,
  N_G4,
  N_A4,
  N_B4,
  N_C5,
  N_D5,
  600
};

const uint8_t rickroll_notes[11] PROGMEM = {
  NI_C3, NI_D3, NI_F3, NI_D3, NI_A3, NI_PAUSE, NI_A3, NI_A3, NI_G3, NI_G3, NI_PAUSE
};
const uint8_t rickroll_durs[11] PROGMEM = {
  16,    16,    16,    16,    8,     16,       16,    8,     8,     4,     16
};
music_t rickroll = {rickroll_notes, rickroll_durs, 4000, 2, 11};

const uint8_t megalovania_notes[344] PROGMEM = {
  NI_D3, NI_D3, NI_D4, NI_A3, NI_PAUSE, NI_GS3, NI_G3, NI_F3, NI_D3, NI_F3, NI_G3, NI_C3, NI_C3, NI_D4, NI_A3, NI_PAUSE, NI_GS3, NI_G3, NI_F3, NI_D3, NI_F3, NI_G3, NI_B2, NI_B2, NI_D4, NI_A3, NI_PAUSE, NI_GS3, NI_G3, NI_F3, NI_D3, NI_F3, NI_G3, NI_AS2, NI_AS2, NI_D4, NI_A3, NI_PAUSE, NI_GS3, NI_G3, NI_F3, NI_D3, NI_F3, NI_G3, NI_D3, NI_D3, NI_D4, NI_A3, NI_PAUSE, NI_GS3, NI_G3, NI_F3, NI_D3, NI_F3, NI_G3, NI_C3, NI_C3, NI_D4, NI_A3, NI_PAUSE, NI_GS3, NI_G3, NI_F3, NI_D3, NI_F3, NI_G3, NI_B2, NI_B2, NI_D4, NI_A3, NI_PAUSE, NI_GS3, NI_G3, NI_F3, NI_D3, NI_F3, NI_G3, NI_AS2, NI_AS2, NI_D4, NI_A3, NI_PAUSE, NI_GS3, NI_G3, NI_F3, NI_D3, NI_F3, NI_G3, NI_D4, NI_D4, NI_D5, NI_A4, NI_PAUSE, NI_GS4, NI_G4, NI_F4, NI_D4, NI_F4, NI_G4, NI_C4, NI_C4, NI_D5, NI_A4, NI_PAUSE, NI_GS4, NI_G4, NI_F4, NI_D4, NI_F4, NI_G4, NI_B3, NI_B3, NI_D5, NI_A4, NI_PAUSE, NI_GS4, NI_G4, NI_F4, NI_D4, NI_F4, NI_G4, NI_AS3, NI_AS3, NI_D5, NI_A4, NI_PAUSE, NI_GS4, NI_G4, NI_F4, NI_D4, NI_F4, NI_G4, NI_D4, NI_D4, NI_D5, NI_A4, NI_PAUSE, NI_GS4, NI_G4, NI_F4, NI_D4, NI_F4, NI_G4, NI_C4, NI_C4, NI_D5, NI_A4, NI_PAUSE, NI_GS4, NI_G4, NI_F4, NI_D4, NI_F4, NI_G4, NI_B3, NI_B3, NI_D5, NI_A4, NI_PAUSE, NI_GS4, NI_G4, NI_F4, NI_D4, NI_F4, NI_G4, NI_AS3, NI_AS3, NI_D5, NI_A4, NI_PAUSE, NI_GS4, NI_G4, NI_F4, NI_D4, NI_F4, NI_G4, NI_F4, NI_F4, NI_F4, NI_F4, NI_F4, NI_D4, NI_D4, NI_D4, NI_F4, NI_F4, NI_F4, NI_G4, NI_GS4, NI_G4, NI_F4, NI_D4, NI_F4, NI_G4, NI_PAUSE, NI_F4, NI_F4, NI_F4, NI_G4, NI_GS4, NI_A4, NI_C5, NI_A4, NI_D5, NI_D5, NI_D5, NI_A4, NI_D5, NI_C5, NI_F4, NI_F4, NI_F4, NI_F4, NI_F4, NI_D4, NI_D4, NI_D4, NI_F4, NI_F4, NI_F4, NI_F4, NI_D4, NI_F4, NI_E4, NI_D4, NI_C4, NI_PAUSE, NI_G4, NI_E4, NI_D4, NI_D4, NI_D4, NI_D4, NI_F3, NI_G3, NI_AS3, NI_C4, NI_D4, NI_F4, NI_C5, NI_PAUSE, NI_F4, NI_D4, NI_F4, NI_G4, NI_GS4, NI_G4, NI_F4, NI_D4, NI_GS4, NI_G4, NI_F4, NI_D4, NI_F4, NI_F4, NI_F4, NI_GS4, NI_A4, NI_C5, NI_A4, NI_GS4, NI_G4, NI_F4, NI_D4, NI_E4, NI_F4, NI_G4, NI_A4, NI_C5, NI_CS5, NI_GS4, NI_GS4, NI_G4, NI_F4, NI_G4, NI_F3, NI_G3, NI_A3, NI_F4, NI_E4, NI_D4, NI_E4, NI_F4, NI_G4, NI_E4, NI_A4, NI_A4, NI_G4, NI_F4, NI_DS4, NI_CS4, NI_DS4, NI_PAUSE, NI_F4, NI_D4, NI_F4, NI_G4, NI_GS4, NI_G4, NI_F4, NI_D4, NI_GS4, NI_G4, NI_F4, NI_D4, NI_F4, NI_F4, NI_F4, NI_GS4, NI_A4, NI_C5, NI_A4, NI_GS4, NI_G4, NI_F4, NI_D4, NI_E4, NI_F4, NI_G4, NI_A4, NI_C5, NI_CS5, NI_GS4, NI_GS4, NI_G4, NI_F4, NI_G4, NI_F3, NI_G3, NI_A3, NI_F4, NI_E4, NI_D4, NI_E4, NI_F4, NI_G4, NI_E4, NI_A4, NI_A4, NI_G4, NI_F4, NI_DS4, NI_CS4, NI_DS4
};
const uint8_t megalovania_durs[344] PROGMEM = {
  16, 16, 8, 6, 32, 8, 8, 8, 16, 16, 16, 16, 16, 8, 6, 32, 8, 8, 8, 16, 16, 16, 16, 16, 8, 6, 32, 8, 8, 8, 16, 16, 16, 16, 16, 8, 6, 32, 8, 8, 8, 16, 16, 16, 16, 16, 8, 6, 32, 8, 8, 8, 16, 16, 16, 16, 16, 8, 6, 32, 8, 8, 8, 16, 16, 16, 16, 16, 8, 6, 32, 8, 8, 8, 16, 16, 16, 16, 16, 8, 6, 32, 8, 8, 8, 16, 16, 16, 16, 16, 8, 6, 32, 8, 8, 8, 16, 16, 16, 16, 16, 8, 6, 32, 8, 8, 8, 16, 16, 16, 16, 16, 8, 6, 32, 8, 8, 8, 16, 16, 16, 16, 16, 8, 6, 32, 8, 8, 8, 16, 16, 16, 16, 16, 8, 6, 32, 8, 8, 8, 16, 16, 16, 16, 16, 8, 6, 32, 8, 8, 8, 16, 16, 16, 16, 16, 8, 6, 32, 8, 8, 8, 16, 16, 16, 16, 16, 8, 6, 32, 8, 8, 8, 16, 16, 16, 8, 16, 8, 8, 8, 8, 4, 16, 8, 16, 8, 8, 8, 16, 16, 16, 16, 16, 8, 8, 16, 8, 8, 8, 8, 8, 8, 8, 8, 16, 16, 16, 2, 8, 16, 8, 8, 8, 8, 4, 16, 8, 16, 8, 8, 8, 8, 8, 16, 8, 16, 8, 8, 8, 8, 8, 8, 8, 16, 8, 15, 8, 8, 2, 3, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 8, 2, 16, 8, 16, 8, 16, 16, 16, 16, 16, 16, 8, 8, 8, 8,  8, 8, 16, 16, 16, 2, 8, 8, 8, 8, 4, 4, 4, 4, 4, 4, 2, 8, 8, 8, 8, 2, 2, 3, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 8, 2, 16, 8, 16, 8, 16, 16, 16, 16, 16, 16, 8, 8, 8, 8,  8, 8, 16, 16, 16, 2, 8, 8, 8, 8, 4, 4, 4, 4, 4, 4, 2, 8, 8, 8, 8, 2, 1
};
music_t megalovania = {megalovania_notes, megalovania_durs, 2000, 2, 344};


uint16_t prevThrottle = 0;
uint16_t prevThrottleRaw = 0;
uint8_t accelerationStarted = 0;
uint8_t throttleOutsideSafetyMarginCount = 0;

uint16_t musicNoteIndex = 0;
uint32_t musicNoteStart = 0;
music_t* curMusic = NULL;

uint8_t curMode = 0;
uint32_t lastModeSelStateChange = 0;
uint8_t lastModeSelState = 0;
uint8_t modeChangePending = 0;

rollingAvg_t currentAverageData;
uint16_t prevCurrent = 0;
int32_t current_integral = 0;
uint32_t prevCurrentControlLoop = 0;

uint32_t lastMainLoop = 0;


void setPwmFreq(uint16_t freq) {
  if (freq < MIN_PWM_FREQ) freq = MIN_PWM_FREQ;
  if (freq > MAX_PWM_FREQ) freq = MAX_PWM_FREQ;
  uint16_t regVal = F_CPU / (TIM1_PRESCALER * 2 * freq);
  OCR1A = regVal;
  //Serial.print("Freq: ");
  //Serial.println(freq);
}

void setPwmDuty(uint8_t duty) {
  uint16_t regVal = OCR1A * (double)(255 - duty) / 255.0;
  OCR1B = regVal;
  //Serial.print("Duty: ");
  //Serial.println(duty);
}

void resetMusic() {
  if (curMusic == NULL) return;
  musicNoteIndex = 0;
  musicNoteStart = millis();
  setPwmFreq(NOTES[pgm_read_byte(curMusic->notes + musicNoteIndex)] * curMusic->freqMultiplier);
}

void setMusic(music_t* music) {
  curMusic = music;
  resetMusic();
}

void playMusic() {
  if (curMusic == NULL) return;
  /*if (millis() - musicNoteStart >= (curMusic->tempo / curMusic->durations[musicNoteIndex]) - 20) {
    setPwmFreq(20000);
    }*/
  if (millis() - musicNoteStart >= (curMusic->tempo / pgm_read_byte(curMusic->durations + musicNoteIndex))) {
    musicNoteIndex++;
    if (musicNoteIndex >= curMusic->length) musicNoteIndex = curMusic->length - 1;
    setPwmFreq(NOTES[pgm_read_byte(curMusic->notes + musicNoteIndex)] * curMusic->freqMultiplier);
    /*Serial.print(musicNoteIndex);
      Serial.print("\t");
      Serial.print(pgm_read_byte(curMusic->notes + musicNoteIndex));
      Serial.print("\t");
      Serial.println(NOTES[pgm_read_byte(curMusic->notes + musicNoteIndex)]);*/
    musicNoteStart = millis();
  }
}

void playRandomNotes(uint16_t tempo) {
  uint16_t duration = map(tempo, 0, 1023, 500, 50);
  if (millis() - musicNoteStart >= duration) {
    setPwmFreq(NOTES[random(NI_A2, NI_PAUSE)]);
    musicNoteStart = millis();
  }
}

void changeMode() {
  curMode++;
  if (curMode >= NUM_MODES) curMode = 0;

  switch (curMode) {
    case 0: {
        // 20 kHz PWM
        setPwmFreq(20000);
        break;
      }

    case 5: {
        // Music: Rickroll
        setMusic(&rickroll);
        break;
      }

    case 6: {
        // Music: Megalovania
        setMusic(&megalovania);
        break;
      }
  }
}

int32_t currentControlLoop(uint16_t current, uint16_t setpoint) {
  int32_t err = (int32_t)setpoint - (int32_t)current;
  current_integral += err;
  if (current_integral >  CURRENT_CONTROL_LOOP_I_LIMIT) current_integral =  CURRENT_CONTROL_LOOP_I_LIMIT;
  if (current_integral < -CURRENT_CONTROL_LOOP_I_LIMIT) current_integral = -CURRENT_CONTROL_LOOP_I_LIMIT;
  prevCurrent = current;
  uint32_t now = micros();
  double dt = (now - prevCurrentControlLoop) / 1000.0;
  prevCurrentControlLoop = now;
  return CURRENT_CONTROL_LOOP_KP * err + (CURRENT_CONTROL_LOOP_KI * current_integral * dt);
}

int32_t rateLimit(int32_t in, int32_t prev, int32_t maxIncrease, int32_t maxDecrease) {
  int32_t diff = in - prev;
  if (diff >= 0) {
    if (diff <= maxIncrease) return in;
    if (diff >  maxIncrease) return prev + maxIncrease;
  } else {
    if (diff >= -maxDecrease) return in;
    if (diff <  -maxDecrease) return prev - maxDecrease;
  }
}

uint16_t rollingAverage(uint16_t value, rollingAvg_t* avgData) {
  avgData->values[avgData->nextIndex] = value;
  avgData->nextIndex++;
  if (avgData->nextIndex >= avgData->len) avgData->nextIndex = 0;
  uint32_t sum = 0;
  for (uint16_t i = 0; i < avgData->len; i++) {
    sum += avgData->values[i];
  }
  return sum / avgData->len;
}


void setup() {
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_MODE_SEL, INPUT_PULLUP);
  pinMode(PIN_BRAKE, INPUT_PULLUP);
  pinMode(PIN_THROTTLE, INPUT);
  pinMode(PIN_CUR_SENS, INPUT);
  pinMode(PIN_LED, OUTPUT);

  digitalWrite(PIN_LED, 0);

  setPwmFreq(20000);
  setPwmDuty(0);

  // PB2 (OC1B, D10) = PWM output, non-inverted; Phase and frequency correct PWM mode, TOP = OCR1A
  TCCR1A = 0b00110001;

  // Phase and frequency correct PWM mode, TOP = OCR1A; Clock prescaler = 1
  TCCR1B = 0b00010001;

  // No force output compare
  TCCR1C = 0b00000000;

  lastModeSelState = digitalRead(PIN_MODE_SEL);
  memset(currentAverageData.values, 0x00, 20);
  currentAverageData.nextIndex = 0;
  currentAverageData.len = 10;

  //Serial.begin(115200);
}

void loop() {
  uint32_t nowMicros = micros();
  if (nowMicros - lastMainLoop < MAIN_LOOP_MIN_INTERVAL_US) return;
  lastMainLoop = nowMicros;

  uint16_t throttle = analogRead(PIN_THROTTLE);
  uint16_t current = rollingAverage(analogRead(PIN_CUR_SENS), &currentAverageData);
  if (current < 123) current = 123;
  current = map(current, 123, 941, 0, 100000); // in milliamps

  // Bit of trickery to filter false signals from noise.
  // Only trigger a mode change if the debounce time has passed without any state changes.
  uint8_t modeSelState = digitalRead(PIN_MODE_SEL);
  if (modeSelState != lastModeSelState) {
    lastModeSelState = modeSelState;
    lastModeSelStateChange = millis();
    if (modeSelState == 1) modeChangePending = 1; // On rising edge
  }
  if (modeChangePending && (millis() - lastModeSelStateChange >= MODE_SEL_DEBOUNCE_TIME)) {
    modeChangePending = 0;
    changeMode();
  }

  if (throttle < THROTTLE_MIN - THROTTLE_SAFETY_MARGIN || throttle > THROTTLE_MAX + THROTTLE_SAFETY_MARGIN) {
    throttleOutsideSafetyMarginCount++;
    if (throttleOutsideSafetyMarginCount >= THROTTLE_OUTSIDE_SAFETY_MARGIN_THRESHOLD) {
      throttle = 0;
      throttleOutsideSafetyMarginCount = 0;
    }
  } else {
    if (throttle < THROTTLE_MIN) throttle = THROTTLE_MIN;
    if (throttle > THROTTLE_MAX) throttle = THROTTLE_MAX;
    throttle = map(throttle, THROTTLE_MIN, THROTTLE_MAX, 0, 1023);
  }
  throttle = rateLimit(throttle, prevThrottleRaw, THROTTLE_MAX_INCREASE, THROTTLE_MAX_DECREASE);
  prevThrottleRaw = throttle;

  int32_t curCtrlVal = currentControlLoop(current, CURRENT_LIMIT);
  if (curCtrlVal < 0) {
    //Serial.println("ctl<0");
    if (-curCtrlVal <= throttle) {
      //Serial.println("ctl<-thr");
      throttle += curCtrlVal;
    } else {
      //Serial.println("thr=0");
      throttle = 0;
    }
  }

  /*Serial.print("THR:");
    Serial.print(throttle);

    Serial.print("\tCUR:");
    Serial.print(current);

    Serial.print("\tCTL:");
    Serial.print(curCtrlVal);

    Serial.print("\tBRK:");
    Serial.println(!digitalRead(PIN_BRAKE));*/

  if (!digitalRead(PIN_BRAKE)) throttle = 0;

  // accelerationStarted is set for the one loop cycle in which the throttle has become non-zero
  if (throttle != 0 && accelerationStarted) accelerationStarted = 0;
  if (prevThrottle == 0 && throttle != 0  ) accelerationStarted = 1;

  if (throttle != 0) {
    switch (curMode) {
      case 0: {
          // 20 kHz PWM
          // Nothing to do
          break;
        }

      case 1: {
          // ICE 3
          if (throttle < 200) {
            setPwmFreq(234);
          } else if (200 <= throttle && throttle < 600) {
            setPwmFreq(map(throttle, 200, 600, 234, 692));
          } else {
            setPwmFreq(692);
          }
          break;
        }

      case 2: {
          // Taurus
          setPwmFreq(freqs_taurus[map(throttle, 0, 1023, 0, 16)]);
          break;
        }

      case 3: {
          // NYC Subway R188 (Bombardier MITRAC)
          if (throttle < 200) {
            setPwmFreq(500);
          } else if (200 <= throttle && throttle < 600) {
            setPwmFreq(map(throttle, 200, 600, 500, 1000));
          } else {
            setPwmFreq(1000);
          }
          break;
        }

      case 4: {
          // Random notes
          if (accelerationStarted) randomSeed(micros());
          playRandomNotes(throttle);
        }

      default: {
          // Music; which music to play has been set in the ISR
          if (accelerationStarted) resetMusic();
          playMusic();
        }
    }
  }

  setPwmDuty(throttle >> 2);
  prevThrottle = throttle;
}
