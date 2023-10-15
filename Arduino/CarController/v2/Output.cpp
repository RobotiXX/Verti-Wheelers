#include "def.h"
#include "CarController.h"
#include "util.h"
#include "Output.h"
#include <Servo.h>

Servo steeringServo;
Servo ESC ;
Servo gearServo;
Servo frontTLockServo;
Servo rearTLockServo;
Servo cameraServo;

int prev_error1 = 0;
int prev_error2 = 0;

int servo_values[6] = {0, 0, 0, 0, 0, 0};

void initServos(){
  // Attach the servos to actual pins
  steeringServo.attach(st_svr);
  ESC.attach(esc); 
  gearServo.attach(g_svr);
  frontTLockServo.attach(fl_svr);
  rearTLockServo.attach(rl_svr);
  cameraServo.attach(cm_svr);

  
  servo_values[0] = 1500;
  servo_values[1] = 1500;
  servo_values[2] = 1800;
  servo_values[3] = 1800;
  servo_values[4] = 1200;
  servo_values[5] = cameraLow;


  // Initialize Servos and ESC setting
  // Steering centered is 90, throttle at neutral is 90 ,Lo gear is 0, front differential lock is 0, rear differential lock is 0, Camera angle is 0
  steeringServo.writeMicroseconds(1500);
  ESC.writeMicroseconds(1500);
  gearServo.writeMicroseconds(1800);
  frontTLockServo.writeMicroseconds(1800);
  rearTLockServo.writeMicroseconds(1200);
  cameraServo.writeMicroseconds(cameraHigh);
  delay(500);
  cameraServo.writeMicroseconds(cameraLow);
  delay(300);

}

void writeServos() {
  steeringServo.writeMicroseconds(servo_values[0]);
  ESC.writeMicroseconds(servo_values[1]);
  gearServo.writeMicroseconds(servo_values[2]);
  frontTLockServo.writeMicroseconds(servo_values[3]);
  rearTLockServo.writeMicroseconds(servo_values[4]);
  cameraServo.writeMicroseconds(servo_values[5]);
}

void failSafeActive(){
  ESC.writeMicroseconds(1500);
  steeringServo.writeMicroseconds(1850);
}

int throttlePID(int c_rpm, int set_speed){
  int esc_out = 0;
  float output = 0;
  int error = set_speed - c_rpm;
  prev_error1 = prev_error1 +  error;
  prev_error2 = error;
  output = int(set_speed + kp * error + ki * (prev_error1 + error) + kd*(error - prev_error2));
  
  if (prev_error1 < -70){
    prev_error1 = -70;
  }

  if(prev_error1 > 180){
    prev_error1 = 180;
  }

  esc_out = (int)fmap(output, 0.0, 100.0, 0.0, 490.0) ;
  if (esc_out < 0){
    esc_out = 0;
  }
  if (esc_out > 490){
    esc_out = 490;
  }
  return esc_out;  
}
