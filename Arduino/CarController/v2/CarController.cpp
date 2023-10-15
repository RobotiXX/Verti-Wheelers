
#include "def.h"
#include "Sensors.h"
#include "Output.h"
#include "CarController.h"
#include "Arduino.h"
#include "util.h"
#define USB_USBCON
#include "CarController.h"

ros::NodeHandle nodeHandle;

unsigned long pidtask = 0;
unsigned long rpmtask = 0;
unsigned long flowtask = 0;
unsigned long last_msg_time = 0;

float Throttle_cmd;
float smt_Throttle;

bool flowErr1 = 0; 
bool flowErr2 = 0;


void driveCallback( const std_msgs::Float32MultiArray&  control_msg ){
  //timestamp the  last ros message
  last_msg_time = millis();

  //Handle for steering command
  //Map steering command to servo output
  float steer_cmd = fmap(control_msg.data[0], -1.0, 1.0, minSteering, maxSteering);
  if (steer_cmd < minSteering) {
    steer_cmd = minSteering;
  }
  if (steer_cmd > maxSteering) {
    steer_cmd = maxSteering ;
  }
  //write Servo output for steering
  servo_values[0] = steer_cmd;

  //Handle for throttle command
  if ((control_msg.data[1] == 0)){
    Throttle_cmd = 0;
    smt_Throttle = 0;
  }
  // ESC forward continue
  if ((control_msg.data[1] >= 0) && (prev_dir == 0)){
    Throttle_cmd = control_msg.data[1];
    smt_Throttle = smt_Throttle + 0.2 * (Throttle_cmd - smt_Throttle);
  }
  
  //ESC reverse continue 
  if ((control_msg.data[1] < 0 ) && (prev_dir == 1)){
    Throttle_cmd = control_msg.data[1];
    smt_Throttle = smt_Throttle + 0.2 * (Throttle_cmd - smt_Throttle);
  }

  //From forward to rev
  if ((control_msg.data[1] < 0 ) && (prev_dir == 0)){
    Throttle_cmd = fw_to_rev(control_msg.data[1]);
    smt_Throttle = Throttle_cmd;
  }

  //From rev to forward
  if ((control_msg.data[1] > 0 ) && (prev_dir == 1)){ 
    Throttle_cmd = rev_to_fw();
    smt_Throttle = Throttle_cmd;
  }


  //Handle for Gear command
  float gearPose = fmap(control_msg.data[3], 0.0, 1.0, servoLow, servoHigh);
  if (gearPose < servoLow) {
    gearPose = servoLow;
  }
  if (gearPose > servoHigh) {
    gearPose = servoHigh ;
  }
  servo_values[2] = gearPose;

  //Handle for Front Differential Lock command
  float fTLockPose = fmap(control_msg.data[4], 0.0, 1.0, servoLow, servoHigh);
  if (fTLockPose < servoLow) {
    fTLockPose = servoLow;
  }
  if (fTLockPose > servoHigh) {
    fTLockPose = servoHigh ;
  }
  servo_values[3] = fTLockPose;

  //Handle for Rear Differential Lock command
  float rTLockPose = fmap(control_msg.data[5], 0.0, 1.0, servoLow, servoHigh);
  if (rTLockPose < servoLow) {
    rTLockPose = servoLow;
  }
  if (rTLockPose > servoHigh) {
    rTLockPose = servoHigh ;
  }  
  servo_values[4] = rTLockPose;

  //Handle for Camera angle command
  float cameraAngle = fmap(control_msg.data[6], 0.0, 1.0, cameraLow, cameraHigh);
  if (cameraAngle < cameraLow) {
    cameraAngle = cameraLow;
  }
  if (cameraAngle > cameraHigh) {
    cameraAngle = cameraHigh ;
  } 
  servo_values[5] = cameraAngle;
}

//Control message subscriber
ros::Subscriber<std_msgs::Float32MultiArray> driveSubscriber("/cmd_vel1", &driveCallback);

//rpmVal_data Publisher for sensor readings
std_msgs::Int16MultiArray rpmVal;
ros::Publisher rpmVal_data("rpmVal_data", &rpmVal);

//optiFlow_data Publisher for sensor readings
std_msgs::Int16MultiArray optiFlow;
ros::Publisher optiFlow_data("optiFlow_data", &optiFlow);

void setup() {
  
  Serial.begin(57600);

  //ROS initialization
  nodeHandle.initNode();
  nodeHandle.subscribe(driveSubscriber);
  nodeHandle.advertise(rpmVal_data);
  nodeHandle.advertise(optiFlow_data);

  //Sensor Initialization
  flowErr1 = pm3901Init();
  flowErr2 = VL53L1xInit();
  rpmInit();

  //Servo Initialization
  initServos();
  Serial.println("Initialization Finished");
}

void loop (){
  unsigned long cur_millis = millis();
  int velocity;
  //Activate Failsafe
  if((millis() - last_msg_time) > 1000){
    failSafeActive();
    pidtask = cur_millis;
  }

  
  //PID task
  else{
    
    if(cur_millis - pidtask >= pidTimer){
      updateMotorRpm();
      velocity = (int)fmap(abs(smt_Throttle), 0, 1, 0, max_speed);
      velocity = (throttlePID(max(m_rpm[0],m_rpm[1]), velocity) * (smt_Throttle/abs(smt_Throttle)));
      servo_values[1] = 1500 + velocity;
      if(servo_values[1] < minThrottle){
        servo_values[1] = minThrottle;
      }
      if(servo_values[1] > maxThrottle){
        servo_values[1] = maxThrottle;
      }
      writeServos();
      pidtask = cur_millis;
  
    }
  }
  
  //RPM Publisher task
  if(cur_millis - rpmtask >= rpmPubTimer){
    rpmVal.data_length = 6;
    int16_t r_packet[6] = {0, 0, 0, 0, 0, 0};
    updateWheelRpm();
    r_packet[0] = w_rpm[0];
    r_packet[1] = w_rpm[1];
    r_packet[2] = w_rpm[2];
    r_packet[3] = w_rpm[3];
    r_packet[4] = m_rpm[0];
    r_packet[5] = (int16_t)(smt_Throttle/abs(smt_Throttle));
    //r_packet[5] = (int16_t)servo_values[1];
    rpmVal.data = r_packet;
    rpmVal_data.publish(&rpmVal);
    rpmtask = cur_millis;   
  }

  //Optiflow publisher task
  if(cur_millis - flowtask >= flowPubTimer ){
    optiFlow.data_length = 5;
    int16_t f_packet[5] = {0, 0, 0, 1, 1};
    readDisplacement();
    int flagz = readZHieght();
    if(!flowErr1){
      f_packet[0] = deltaX;
      f_packet[1] = deltaY;
      f_packet[3] = 0;
    }
    if(!flowErr2){
      f_packet[2] = distZ;
      f_packet[4] = flagz;
    }
    optiFlow.data = f_packet;
    optiFlow_data.publish( &optiFlow);
    flowtask = cur_millis;
  }
  
  nodeHandle.spinOnce();
  delayMicroseconds(100);
    
}
