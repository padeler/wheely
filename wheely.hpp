#pragma once
#include <EEPROM.h>
// #include <avr/iom32u4.h>
// Pin definition
#define EN_A 5 //9
#define IN1_A 18
#define IN2_A 19

#define IN1_B 20
#define IN2_B 21
#define EN_B 6 //10

#define MAX_ANGLE 50
// Good values for 3s 
// #define MIN_POWER 120
// #define MAX_POWER 250
// Good values for 2s 
#define MIN_POWER 140
#define MAX_POWER 255


#define PID_TARGET_RANGE 13.0

#define PID_SAMPLE_TIME 5


#define UP(x) (uint16_t)(0x0100|(uint16_t)x)
#define NOIN 0b00000000
#define TLD  0b00000001
#define TRD  0b00000010
#define BLD  0b00000100
#define BRD  0b00001000
#define TD   0b00010000
#define BD   0b00100000
#define LD   0b01000000
#define RD   0b10000000

#define TLU UP(TLD)
#define TRU UP(TRD)
#define BLU UP(BLD)
#define BRU UP(BRD)
#define TU  UP(TD)
#define BU  UP(BD)
#define LU  UP(LD)
#define RU  UP(RD)


#define INPUT_FLAG(tl,tr,bl,br, t, b, l, r) (tl|tr<<1|bl<<2|br<<3|t<<4|b<<5|l<<6|r<<7)

class InputHandler
{
  public:
  InputHandler(float thres=0.2):thres(thres),
    tl(false), tr(false), bl(false), br(false),t(false), b(false), l(false), r(false)
  {
    last_check=0;
    check_period=200;
  }

  uint16_t operator()(float th, float st){
    long now = millis();
    if(now-last_check>check_period)
    {
      last_check = now;
      th = th*2.0 -1.0;
      st = st*2.0 -1.0;

      bool left = st-thres<=-1.0; 
      bool right = st+thres>=1.0;
      bool bottom = th-thres<=-1.0;
      bool top = th+thres>=1.0;

      bool th_center = th > -thres && th < thres;
      bool st_center = st > -thres && st < thres;
      
      if(th_center && st_center)
      {
        uint16_t flag = INPUT_FLAG(tl, tr, bl, br, t ,b, l, r);
        if(flag)
        { // report key up
          tl=false; tr=false; bl=false; br=false; t=false; b=false, l=false, r=false;
          return UP(flag);
        } // else flags will be reset bellow:
      }
      tl = top && left;
      tr = top && right;
      bl = bottom && left;
      br = bottom && right;
      t = top && !left && !right;
      b = bottom && !left && !right;
      l = left && !top && !bottom;
      r = right && !top && !bottom;

      return INPUT_FLAG(tl, tr, bl, br, t, b, l, r);
    }
    return NOIN;
  }

  private:
  bool tl, tr, bl, br, t, b, l, r;
  float thres;
  long last_check;
  uint16_t check_period;
};

/*   - Base frequencies:
 *      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
 *      o The base frequency for pins 5 and 6 is 62500 Hz.
 */
void setPwmFrequency(int pin, int divisor){
  byte mode;
  // PIN 5 is connected to timer3 with 3 CS bits
 	//     The three Clock Select bits select the clock source to be used by the
	//     Timer/Counter.
	//
	//     CSn2 CSn1 CSn0  Description
	//        0    0    0          OFF
	//        0    0    1        clock
	//        0    1    0    clock / 8
	//        0    1    1   clock / 64
	//        1    0    0  clock / 256
	//        1    0    1 clock / 1024 
  // See Table 13-8 in the datasheet (below).
  // 
  // PIN 6 is connected to timer4 with 4 CS bits
  // See Table 15-14 on 32u4 datasheet:
  // https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7766-8-bit-AVR-ATmega16U4-32U4_Datasheet.pdf
  if(pin==5)
  {
    switch(divisor){
      case 1:mode=0x01;break;
      case 8:mode=0x02;break;
      case 64:mode=0x03;break;
      case 256:mode=0x04;break;
      case 1024:mode=0x05;break;
      default:return;
    }
    TCCR3B=TCCR3B&0b11111000|mode;
  }
  else{
    switch(divisor){
      case 1:mode=0x01;break;
      case 8:mode=0x04;break;
      case 64:mode=0x07;break;
      case 256:mode=0x09;break;
      case 1024:mode=0x0B;break;
      default:return;
    }
    TCCR4B=TCCR4B&0b11111000|mode;
  }
}

#define OFFSETS_FLAG 123
#define OFFSETS_FLAG_IDX 0

bool Read_MPUOffsets(float &offX, float &offY, float &offZ, float &pid_target)
{
  byte flag;
  flag = EEPROM.read(OFFSETS_FLAG_IDX);
  if(flag==OFFSETS_FLAG)
  { // There are data in the EEPROM
     EEPROM.get(OFFSETS_FLAG_IDX+1, offX);
     EEPROM.get(OFFSETS_FLAG_IDX+1+sizeof(float), offY);
     EEPROM.get(OFFSETS_FLAG_IDX+1+2*sizeof(float), offZ);
     EEPROM.get(OFFSETS_FLAG_IDX+1+3*sizeof(float), pid_target);
     return true;
  }
  // No data, return defaults
  offX = -2.17;
  offY = 2.44;
  offZ = 0.16;
  pid_target=0;
  return false;
}

void Write_MPUOffsets(float offX, float offY, float offZ, float pid_target)
{
  EEPROM.update(OFFSETS_FLAG_IDX, OFFSETS_FLAG);
  EEPROM.put(OFFSETS_FLAG_IDX+1, offX);
  EEPROM.put(OFFSETS_FLAG_IDX+1+sizeof(float), offY);
  EEPROM.put(OFFSETS_FLAG_IDX+1+2*sizeof(float), offZ);
  EEPROM.put(OFFSETS_FLAG_IDX+1+3*sizeof(float), pid_target);
}