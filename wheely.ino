#include "wheely.hpp"
#include "math.h"

#include <PID_v1.h>
#include <L298NX2.h>
#include <ServoInput.h>
#include <MPU6050_tockn.h>
#include <Wire.h>

#include "tunes.hpp"

int count = 0;
int loop_time = 0;
unsigned long last_update = 0;

double pid_in, pid_out, pid_target = 0;
double kp = 10, ki = 80, kd = 0.25; // good trimming for hi freq PWMs (490hz?)
//double kp = 12, ki = 150, kd = 0.15; 

bool serial_connected = false;

bool robot_down = false;

PID pid(&pid_in, &pid_out, &pid_target, kp, ki, kd, P_ON_E, REVERSE);

// Initialize both motors
L298NX2 motors(EN_A, IN1_A, IN2_A, EN_B, IN1_B, IN2_B);

MPU6050 mpu6050(Wire);


/* Signal pins for ServoInput MUST be interrupt-capable pins!
 *     Uno, Nano, Mini (328P): 2, 3
 *     Micro, Leonardo (32U4): 0, 1, 2, 3, 7
 *             Mega, Mega2560: 2, 3, 18, 19, 20, 21
 * Reference: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
 */
// Steering Setup
const int SteeringSignalPin = 1;  // MUST be interrupt-capable!
const int SteeringPulseMin = 948;  // microseconds (us)
const int SteeringPulseMax = 2008;  // Ideal values for your servo can be found with the "Calibration" example

ServoInputPin<SteeringSignalPin> steering(SteeringPulseMin, SteeringPulseMax);

// Throttle Setup
const int ThrottleSignalPin = 0;  // MUST be interrupt-capable!
const int ThrottlePulseMin = 960;  // microseconds (us)
const int ThrottlePulseMax = 2000;  // Ideal values for your servo can be found with the "Calibration" example

ServoInputPin<ThrottleSignalPin> throttle(ThrottlePulseMin, ThrottlePulseMax);

// Pot Setup
const int Ch6SignalPin = 7;  // MUST be interrupt-capable!
const int Ch6PulseMin = 1040;  // microseconds (us)
const int Ch6PulseMax = 1932;  // Ideal values for your servo can be found with the "Calibration" example

ServoInputPin<Ch6SignalPin> pot(Ch6PulseMin, Ch6PulseMax);

TunePlayer player;
InputHandler input;

#define T(m) Tune(LEN(m),m)
int tune_select = 0;
const Tune tunes[] = {
  T(pacman_melody), T(mario_melody), T(starwars_melody), T(immarch_melody), T(doom_melody)
};


void setup()
{
  Wire.begin();
  mpu6050.begin();

  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(PID_SAMPLE_TIME);
  pid.SetOutputLimits(-MAX_POWER, MAX_POWER);
  pid_target = 0;


  // Used to display information
  Serial.begin(9600);
  serial_connected=true;
  // for(int i=0;i<3;++i)
  // {
  //   delay(200);
  //   if(Serial){
  //     break;
  //   }
  // }

  int wait_count=2;
	while ((wait_count--)>=0) {  // wait for all signals to be ready
		print("Waiting for RC signals...\n");
    if(ServoInput.available())
    {
      ALERT_BLOCK(player, link_up);
      break;
    }
    ALERT_BLOCK(player, wait_link);
    delay(300);
	}

  float offX, offY, offZ;
  if(ServoInput.available() && steering.getPercent()>0.7 && throttle.getPercent()>0.7)
  { 
    ALERT_BLOCK(player, calibrating);
    mpu6050.calcGyroOffsets(true);
    ALERT_BLOCK(player, calibrating);
    // store offsets to EEPROM
    offX = mpu6050.getGyroXoffset();
    offY = mpu6050.getGyroYoffset();
    offZ = mpu6050.getGyroZoffset();
    print("Writing offsets to EEPROM...\n"); 
    Write_MPUOffsets(offX, offY, offZ);
  }
  else
  {
    print("Loading offsets from EEPROM..."); 
    Read_MPUOffsets(offX, offY, offZ);  
  }

  print("Gyro offsets: ");
  print(offX);print(", ");
  print(offY);print(", ");
  print(offZ);print("\n");

  mpu6050.setGyroOffsets(offX, offY, offZ);
  // setPwmFrequency(EN_A, 256);
  // setPwmFrequency(EN_B, 256);

  ALERT(player, ready);

  // TUNE(player, mario_melody);
}

void setMotors(int speedA, int speedB)
{
  speedA = constrain(speedA, -MAX_POWER, MAX_POWER);
  speedB = constrain(speedB, -MAX_POWER, MAX_POWER);
  if(abs(speedA)<MIN_POWER) speedA=0;
  if(abs(speedB)<MIN_POWER) speedB=0;


  motors.setSpeedA(abs(speedA));
  motors.setSpeedB(abs(speedB));

  speedA>=0 ? motors.forwardA(): motors.backwardA();
  speedB>=0 ? motors.forwardB(): motors.backwardB();
}

template< typename T > void print(const T &t )
{
  if(serial_connected)
  {
    Serial.print(t);
  }
}


void loop()
{

    // // while(true){
    // if(Serial)
    // {
    //   Serial.println("Link up.");
    //   play_tone(link_up, LEN(link_up), 100);
    // }
    // else{
		//   play_tone(wait_link, LEN(wait_link), 30);
    //   // delay(1000);
    // }
    // delay(1500);
    // return;


  unsigned long st = millis();
  float in_rot=0, in_throttle=0, in_pot=0;

  if(ServoInput.available())
  {
    in_rot = 2.0 * MAX_POWER * steering.getPercent() - MAX_POWER;
    in_throttle = 2.0 * PID_TARGET_RANGE * throttle.getPercent() - PID_TARGET_RANGE;
    in_pot = 2.0 * PID_TARGET_RANGE * pot.getPercent() - PID_TARGET_RANGE;

    pid_target = in_pot;

    if(abs(in_throttle)>1.0) // above throttle dead zone
    {
      pid_target += in_throttle;
    }
  }

  if (st - last_update >= PID_SAMPLE_TIME)
  {
    mpu6050.update();
    last_update = st;
    pid_in = mpu6050.getAngleX();
  }

  if (abs(pid_in) < MAX_ANGLE) // if not fallen
  {
    if(robot_down){
      robot_down=false;
      player.pause_tune(false);
      ALERT(player, ready);
    }
    if (pid.Compute())
    {
      // set motor power after constraining it
      // motorPower = constrain(motorPower, -255, 255);
      int motorA = pid_out, motorB = pid_out;

      int rot = int(abs(in_rot));
      if(in_rot>=0)
      {
        motorA+=rot; motorB-=rot;
      }
      else
      {
        motorA-=rot; motorB+=rot;
      }

      setMotors(motorA, motorB);

    }
  }
  else
  {
    pid_target = 0;
    setMotors(0, 0);
    if(!robot_down)
    {
      robot_down=true;
      player.pause_tune(true);
      ALERT(player, fallen);
    }
    else
    {
      uint8_t flag = input(throttle.getPercent(), steering.getPercent());
      // if(flag!=NOIN){
      //   ALERT(player, click);
      //   Serial.print("Input Handler: "); Serial.println(flag, BIN);
      // }

      if(flag==BRU) // stop music
      {
        ALERT(player, click);
        print("Stop Tune.\n");
        player.stop_tune();
      }
      else if(flag==BLU) // next tune
      {
        tune_select = (tune_select+1) % (int)(LEN(tunes));
        player.set_tune(tunes[tune_select]);
        ALERT(player, click);
        print("Playing tune ");print(tune_select);print("\n");
      }
    }
  }

  player(); // process pending tunes

  count += 1;
  loop_time += millis() - st;

  if (loop_time>500) // log but not very often
  {
    float dt = loop_time / count;
    count = 0;
    loop_time = 0;
    print("Av time ");
    print(dt);
    print(" pid_out: ");
    print(pid_out);
    print(" pid_target: ");
    print(pid_target);
    print(" AngleX: ");
    print(pid_in);

    print(" IN R: "); print(in_rot);
    print(" T: "); print(in_throttle);
    print(" Pot: "); print(in_pot);
    print("\n");
  }

}