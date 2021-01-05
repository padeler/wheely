#pragma once

// Pin definition
#define EN_A 5
#define IN1_A 18
#define IN2_A 19

#define IN1_B 20
#define IN2_B 21
#define EN_B 6

#define BUZZER 10 // PIN for buzzer

#define MAX_ANGLE 45
#define MIN_POWER 25
#define MAX_POWER 90

#define PID_TARGET_RANGE 12.0

#define PID_SAMPLE_TIME 10

#define LEN(x) sizeof(x)/sizeof(x[0])

unsigned short fallen[] = {200, 100, 50 , 10};
unsigned short ready[] = {200, 150, 255, 255};
unsigned short calibrating[] = {100, 150, 100, 150, 100, 150, 100, 150};
unsigned short wait_link[] = {20,};
unsigned short link_up[] = {200, 200};


void play_tone(unsigned short *t, int len, unsigned int d=50)
{
  for(int i=0;i<len; ++i)
  {
    analogWrite(BUZZER, t[i]);
    delay(d);
    analogWrite(BUZZER, 0);
    delay(d);
  }
}

class TargetAccum
{
public:
  TargetAccum(int update_interval, float scale) : update_interval(update_interval), scale(scale)
  {
    count = 0;
    accum = 0;
  }

  bool update(float new_val)
  {
    accum = scale * accum + (1.0 - scale) * new_val;
    count += 1;
    if (count >= update_interval)
    {
      count = 0;
    }
    return count == 0;
  }

  float getAccum()
  {
    return accum;
  }

  void reset()
  {
    count = 0;
    accum = 0;
  }

private:
  int update_interval;
  float accum, scale;
  int count;
};
