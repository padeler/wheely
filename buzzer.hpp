#pragma once
#include "wiring_private.h"
#include "pins_arduino.h"


#define BUZZER 10 //6 // PIN for buzzer

#define NOTE_B0 31
#define NOTE_C1 33
#define NOTE_CS1 35
#define NOTE_D1 37
#define NOTE_DS1 39
#define NOTE_E1 41
#define NOTE_F1 44
#define NOTE_FS1 46
#define NOTE_G1 49
#define NOTE_GS1 52
#define NOTE_A1 55
#define NOTE_AS1 58
#define NOTE_B1 62
#define NOTE_C2 65
#define NOTE_CS2 69
#define NOTE_D2 73
#define NOTE_DS2 78
#define NOTE_E2 82
#define NOTE_F2 87
#define NOTE_FS2 93
#define NOTE_G2 98
#define NOTE_GS2 104
#define NOTE_A2 110
#define NOTE_AS2 117
#define NOTE_B2 123
#define NOTE_C3 131
#define NOTE_CS3 139
#define NOTE_D3 147
#define NOTE_DS3 156
#define NOTE_E3 165
#define NOTE_F3 175
#define NOTE_FS3 185
#define NOTE_G3 196
#define NOTE_GS3 208
#define NOTE_A3 220
#define NOTE_AS3 233
#define NOTE_B3 247
#define NOTE_C4 262
#define NOTE_CS4 277
#define NOTE_D4 294
#define NOTE_DS4 311
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_FS4 370
#define NOTE_G4 392
#define NOTE_GS4 415
#define NOTE_A4 440
#define NOTE_AS4 466
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_CS5 554
#define NOTE_D5 587
#define NOTE_DS5 622
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_FS5 740
#define NOTE_G5 784
#define NOTE_GS5 831
#define NOTE_A5 880
#define NOTE_AS5 932
#define NOTE_B5 988
#define NOTE_C6 1047
#define NOTE_CS6 1109
#define NOTE_D6 1175
#define NOTE_DS6 1245
#define NOTE_E6 1319
#define NOTE_F6 1397
#define NOTE_FS6 1480
#define NOTE_G6 1568
#define NOTE_GS6 1661
#define NOTE_A6 1760
#define NOTE_AS6 1865
#define NOTE_B6 1976
#define NOTE_C7 2093
#define NOTE_CS7 2217
#define NOTE_D7 2349
#define NOTE_DS7 2489
#define NOTE_E7 2637
#define NOTE_F7 2794
#define NOTE_FS7 2960
#define NOTE_G7 3136
#define NOTE_GS7 3322
#define NOTE_A7 3520
#define NOTE_AS7 3729
#define NOTE_B7 3951
#define NOTE_C8 4186
#define NOTE_CS8 4435
#define NOTE_D8 4699
#define NOTE_DS8 4978
#define REST 0


volatile long timer1_toggle_count;
volatile uint8_t *timer1_pin_port;
volatile uint8_t timer1_pin_mask;

/**
 * Adapted from analogWrite and tone functions of the Arduino SKD
 */
void pwm(unsigned int frequency)
{
  pinMode(BUZZER, OUTPUT);
  if(frequency == 0)
  {
    bitWrite(TIMSK1, OCIE1A, 0);
    digitalWrite(BUZZER, 0);
  }
  else
  {
    // Set the prescaler for the timer
    uint8_t timer = digitalPinToTimer(BUZZER);
    if (timer != TIMER1B)
      return; // only timer1b is implemented

    uint8_t prescalarbits;
    uint32_t ocr;

    // two choices for the 16 bit timers: ck/1 or ck/64
    ocr = F_CPU / frequency / 2 - 1;

    prescalarbits = 0b001;
    if (ocr > 0xffff)
    {
      ocr = F_CPU / frequency / 2 / 64 - 1;
      prescalarbits = 0b011;
    }

    TCCR1A = 0;
    TCCR1B = 0;
    bitWrite(TCCR1B, WGM12, 1);
    bitWrite(TCCR1B, CS10, 1);
    timer1_pin_port = portOutputRegister(digitalPinToPort(BUZZER));
    timer1_pin_mask = digitalPinToBitMask(BUZZER);

    TCCR1B = (TCCR1B & 0b11111000) | prescalarbits;
    // TCCR1A = (TCCR1A & 0b11111000) | prescalarbits;
    // connect pwm to pin on timer 1, channel B
    // sbi(TCCR1A, COM1B1);
    // OCR1B = ocr; // set pwm duty
    OCR1A = ocr;
    timer1_toggle_count = -1;
    bitWrite(TIMSK1, OCIE1A, 1);
  }
}

ISR(TIMER1_COMPA_vect)
{
  if (timer1_toggle_count != 0)
  {
    // toggle the pin
    *timer1_pin_port ^= timer1_pin_mask;

    if (timer1_toggle_count > 0)
      timer1_toggle_count--;
  }
  else
  {
    bitWrite(TIMSK1, OCIE1A, 0);
    *timer1_pin_port &= ~(timer1_pin_mask); // keep pin low after stop
  }
}


struct Tune
{
  Tune() : ready(false) {}

  Tune(int melody_len, const uint16_t *melody PROGMEM) : ready(true)
  {
    notes = (melody_len-1)/2;
    this->melody = melody+1;
    wholenote = (60000 * 4) / pgm_read_word_near(melody); // melody[0] is TEMPO
    noteDuration = 0;
    currentNote = -2;
    currentNoteStart=0;
  }

  bool operator()() { return ready; }

  bool _wait_note()
  {
    if(currentNoteStart==0) return false; // not started playing yet.

    long dt = millis() - currentNoteStart;
    if(dt>=noteDuration*0.9 && !currentNoteStopped)
    {
      pwm(0); // stop note
      currentNoteStopped=true;
    }
    if(dt>=noteDuration)
    { // note is done 
      return false;
    }
    return true;
  }

  bool _next_note()
  {
    currentNote += 2;
    if (currentNote == notes * 2)
    {
      ready = false;
      return ready;
    }

    // calculates the duration of each note
    int divider = pgm_read_word_near(melody + currentNote + 1);
    if (divider > 0)
    {
      // regular note, just proceed
      noteDuration = (wholenote) / divider;
    }
    else if (divider < 0)
    {
      // dotted notes are represented with negative durations!!
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5; // increases the duration in half for dotted notes
    }

    currentNoteStopped=false;
    // start playing the note
    currentNoteStart = millis();
    pwm(pgm_read_word_near(melody+currentNote));
    
    return true;
  }

  const uint16_t *melody PROGMEM;
  int notes, wholenote, noteDuration;
  int currentNote;
  long currentNoteStart;
  bool currentNoteStopped;
  bool ready;
};

class TunePlayer
{
public:
  TunePlayer():paused(false), disabled(false)
  {}

  void set_alert(const Tune &alert)
  {
    this->alert = alert;
  }

  void set_tune(const Tune &tune)
  {
    this->tune = tune;
  }

  void set_disabled(bool disabled){
    this->disabled = disabled;
  }

  /**
   * Pause Tunes. Does not affect alerts
   */
  void pause_tune(bool paused)
  {
    this->paused = paused;
    pwm(0);
  }

  void stop_tune() 
  {
    this->tune = Tune();
    pwm(0);
  }

  void operator()()
  {
    if(disabled) return;

    if (alert())
    {
      _play(alert);
    }
    else if (!paused && tune())
    {
      _play(tune);
    }
  }

  /**
   * Play tune until finished (blocking)
   * Not affected by pause
   */
  void play(Tune t)
  {
    if(disabled) return;

    while(t()) // while there are notes to play
    {
      if(!t._wait_note())
      {
        if(t._next_note())
          delay(t.noteDuration);
      }
    }
  }

private:
  void _play(Tune &t)
  {
      if (!t._wait_note())
      {
        t._next_note();
      }
  }

  Tune tune;
  Tune alert;
  bool paused; // pause tunes (not alerts)
  bool disabled; // mute everything
};
