#pragma once
#include <EEPROM.h>

// Pin definition
#define EN_A 5 //9
#define IN1_A 18
#define IN2_A 19

#define IN1_B 20
#define IN2_B 21
#define EN_B 6 //10

#define MAX_ANGLE 45
#define MIN_POWER 25
#define MAX_POWER 140

#define PID_TARGET_RANGE 12.0

#define PID_SAMPLE_TIME 5


/**
 * TODO
 * Holds the state of the robot and relevant metrics.
 * Implements behaivior (music, calibrate)
 * 
 */
// class Robot
// {

//   public:
//   bool fallen;
//   float pid_in, pid_out, pid_target;

//   // counters and metrics
//   int count;
//   unsigned long loop_ms_sum;
//   float angleX_sum;
//   float pid_out_sum;



// };

#define NOIN 0b00000000
#define TLD 0b00000001
#define TRD 0b00000010
#define BLD 0b00000100
#define BRD 0b00001000

#define TLU 0b10000001
#define TRU 0b10000010
#define BLU 0b10000100
#define BRU 0b10001000

#define INPUT_FLAG(tl,tr,bl,br) (tl|tr<<1|bl<<2|br<<3)

class InputHandler
{
  public:
  InputHandler(float thres=0.2):thres(thres),
    tl(false), tr(false), bl(false), br(false)
  {
    last_check=0;
    check_period=350;
  }

  uint8_t operator()(float th, float st){
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
        uint8_t flag = INPUT_FLAG(tl, tr, bl, br);
        if(flag)
        { // report key up
          tl=false; tr=false; bl=false; br=false;
          return 0b10000000|flag;
        } // else flags will be reset bellow:
      }
      tl = top && left;
      tr = top && right;
      bl = bottom && left;
      br = bottom && right;

      return INPUT_FLAG(tl, tr, bl, br);
    }
    return NOIN;
  }

  private:
  bool tl, tr, bl, br;
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
  switch(divisor){
    case 1:mode=0x01;break;
    case 8:mode=0x02;break;
    case 64:mode=0x03;break;
    case 256:mode=0x04;break;
    case 1024:mode=0x05;break;
    default:return;
  }
  switch(pin){
  case 5:TCCR3A=TCCR3A&0b11111000|mode;break;
  case 6:TCCR4D=TCCR4D&0b11111000|mode;break;
  case 9:TCCR1A=TCCR1A&0b11111000|mode;break;
  case 10:TCCR1B=TCCR1B&0b11111000|mode;break;
  default:return;
  }
}

#define OFFSETS_FLAG 123
#define OFFSETS_FLAG_IDX 0

bool Read_MPUOffsets(float &offX, float &offY, float &offZ)
{
  byte flag;
  flag = EEPROM.read(OFFSETS_FLAG_IDX);
  if(flag==OFFSETS_FLAG)
  { // There are data in the EEPROM
     EEPROM.get(OFFSETS_FLAG_IDX+1, offX);
     EEPROM.get(OFFSETS_FLAG_IDX+1+sizeof(float), offY);
     EEPROM.get(OFFSETS_FLAG_IDX+1+2*sizeof(float), offZ);
     return true;
  }
  // No data, return defaults
  offX = -2.17;
  offY = 2.44;
  offZ = 0.16;
  return false;
}

void Write_MPUOffsets(float offX, float offY, float offZ)
{
  EEPROM.update(OFFSETS_FLAG_IDX, OFFSETS_FLAG);
  EEPROM.put(OFFSETS_FLAG_IDX+1, offX);
  EEPROM.put(OFFSETS_FLAG_IDX+1+sizeof(float), offY);
  EEPROM.put(OFFSETS_FLAG_IDX+1+2*sizeof(float), offZ);
}