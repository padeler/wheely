#pragma once

// Pin definition
#define EN_A 9
#define IN1_A 18
#define IN2_A 19

#define IN1_B 20
#define IN2_B 21
#define EN_B 10

#define BUZZER 6 // PIN for buzzer

#define MAX_ANGLE 60
#define MIN_POWER 25
#define MAX_POWER 90

#define PID_TARGET_RANGE 12.0

#define PID_SAMPLE_TIME 10

#define LEN(x) sizeof(x)/sizeof(x[0])

/**** Notification Tones  ****/
unsigned short fallen[] = {200, 100, 50 , 10};
unsigned short ready[] = {200, 150, 255, 255};
unsigned short calibrating[] = {100, 150, 100, 150, 100, 150, 100, 150};
unsigned short wait_link[] = {20,};
unsigned short link_up[] = {200, 200};
/******************************/

void play_tone(unsigned short *t, int len, unsigned int d=50)
{
  return;
  for(int i=0;i<len; ++i)
  {
    analogWrite(BUZZER, t[i]);
    delay(d);
    analogWrite(BUZZER, 0);
    delay(d);
  }
}

/*   - Base frequencies:
 *      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
 *      o The base frequency for pins 5 and 6 is 62500 Hz.
 */
void setPwmFrequency(int pin, int divisor){
  byte mode;
  if(pin==5||pin==6||pin==9||pin==10){
    switch(divisor){
      case 1:mode=0x01;break;
      case 8:mode=0x02;break;
      case 64:mode=0x03;break;
      case 256:mode=0x04;break;
      case 1024:mode=0x05;break;
      default:return;
    }
    if(pin==5||pin==6){
      TCCR0B=TCCR0B&0b11111000|mode;
    }else{
      TCCR1B=TCCR1B&0b11111000|mode;
    }
  }
}