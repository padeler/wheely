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

double pid_in, pid_out, pid_target = 0, av_pid_out=0;
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

  int wait_count=2;
	while ((wait_count--)>=0) {  // wait for all signals to be ready
		log("Waiting for RC signals...\n");
    if(ServoInput.available())
    {
      ALERT_BLOCK(player, link_up);
      break;
    }
    ALERT_BLOCK(player, wait_link);
    delay(300);
	}

  float offX, offY, offZ;
  if(ServoInput.available() && input(throttle.getPercent(), steering.getPercent())==TRD)
  { 
    ALERT_BLOCK(player, calibrating);
    mpu6050.calcGyroOffsets(true);
    ALERT_BLOCK(player, calibrating);
    // store offsets to EEPROM
    offX = mpu6050.getGyroXoffset();
    offY = mpu6050.getGyroYoffset();
    offZ = mpu6050.getGyroZoffset();
    log("Writing offsets to EEPROM...\n"); 
    Write_MPUOffsets(offX, offY, offZ);
  }
  else
  {
    log("Loading offsets from EEPROM..."); 
    Read_MPUOffsets(offX, offY, offZ);  
  }

  log("Gyro offsets: ");
  log(offX);log(", ");
  log(offY);log(", ");
  log(offZ);log("\n");

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

template< typename T > void log(const T &t )
{
  if(serial_connected)
  {
    Serial.print(t);
  }
}

double speed_sigmoid(double x, double a=30, double b=15)
{
  return 1 - 1/(1+exp(-(a*x-b)));
}

float in_rot=0, in_throttle=0, in_pot=0;

void loop()
{
  unsigned long st = millis();

  if(ServoInput.available())
  {
    in_rot = 2.0 * MAX_POWER * steering.getPercent() - MAX_POWER;
    in_throttle = 2.0 * PID_TARGET_RANGE * throttle.getPercent() - PID_TARGET_RANGE;
    in_pot = 2.0 * PID_TARGET_RANGE * pot.getPercent() - PID_TARGET_RANGE;

    pid_target = in_pot;

    uint8_t flag = input(throttle.getPercent(), steering.getPercent());
    if(flag!=NOIN){
      ALERT(player, click);
      Serial.print("Input Handler: "); Serial.println(flag, BIN);
    }

    if(flag==BRU) // stop music
    {
      ALERT(player, click2);
      log("Stop Tune.\n");
      player.stop_tune();
    }
    else if(flag==BLU) // next tune
    {
      tune_select = (tune_select+1) % (int)(LEN(tunes));
      player.set_tune(tunes[tune_select]);
      ALERT(player, click);
      log("Playing tune ");log(tune_select);log("\n");
    }
    else if(flag==TRD && fallen)
    {
      int speed = 100;
      if(pid_in<0) // robot on its back
      {
        speed=-speed;
      }
      Serial.println("Self-Raise Sequence.");
      setMotors(speed, speed);
    }


    if(abs(in_throttle)>1.0) // above throttle dead zone
    {  
      float scale=1.0;
      if(in_throttle*av_pid_out>0) // throttle is in the direction of motion
      { // scale adjustment to avoid falling due to overpowering the motors
        // XXX This is a bad aproximation (see av_pid_out below).
        scale = speed_sigmoid(abs(av_pid_out)/MAX_POWER);
      }// if throttle is the oposite of the direction of motion no scaling is needed (so scale==1).
      pid_target += in_throttle * scale;
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
      // XXX This is a bad aproximation. 
      // The correct way to compensate for 
      // motor speed is to use odometry (encoders).
      av_pid_out = 0.99*av_pid_out + 0.01*pid_out; 

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
    if(!robot_down)
    {
      setMotors(0, 0);
      av_pid_out = 0;
      robot_down=true;
      player.pause_tune(true);
      ALERT(player, fallen);
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
    log("Av time ");
    log(dt);
    log(" pid_out: ");
    log((float)pid_out);
    log(" pid_target: ");
    log((float)pid_target);
    log(" AngleX: ");
    log((float)pid_in);

    log(" IN R: "); log(in_rot);
    log(" T: "); log(in_throttle);
    log(" Pot: "); log(in_pot);
    log("\n");
  }

}