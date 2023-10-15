

/*
  Arduino ROS node for JetsonCar project
  The Arduino controls a TRAXXAS Rally Car
  MIT License
  JetsonHacks (2016)
*/

/*
  Modified by Aniket Datar, Chenhui Pan
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include <WProgram.h>
#endif

//Flowdeck V2 include files
#include <Servo.h>
#include <Bitcraze_PMW3901.h>
#include <Wire.h>
#include <VL53L1X.h>


//ROS include files
#define USB_USBCON
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>

//servo pin defination
#define st_svr 2   //Steering Servo 
#define esc 3      //Electronic Speed Controller
#define g_svr 4    //Hi-LO gear servo
#define fl_svr 5   //Front Differential Lock Servo
#define rl_svr 6   //Rear Differential Lock Servo
#define cm_svr 7   //Camera Tilt Servo

//RPM sensor reading pin and interrupt definations
#define PCINT_PIN_COUNT            4
#define PCINT_RX_BITS              (1<<0),(1<<1),(1<<2),(1<<3)
#define PCINT_RX_PORT              PORTK
#define PCINT_RX_MASK              PCMSK2
#define PCIR_PORT_BIT              (1<<2)
#define RX_PC_INTERRUPT            PCINT2_vect
#define RX_PCINT_PIN_PORT          PINK
#define rpmsc                      4
static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS};
volatile uint16_t rpmValue[4] = {0,0,0,0};
int rpm[4] = {0,0,0,0};

//Flowdeck V2 object and variables
Bitcraze_PMW3901 flow(53);
VL53L1X sensor;
int flowVal[5] = {0,0,0,-1,-1}; 
int16_t deltaX,deltaY;
int distZ = 0;
int flsf1 = 0;
int flsf2 = 0;

unsigned long previousMillis1 = 0;
unsigned long previousMillis2 = 0;
unsigned long last_msg_time = 0;

ros::NodeHandle nodeHandle;

// Limits for the servo range
const float minSteering = 1150 ;
const float maxSteering = 1850 ;
const float minThrottle = 1200 ;
const float maxThrottle = 1900 ;
const float servoLow = 1050;
const float servoHigh = 1850;
const float cameraLow = 620;
const float cameraHigh = 2200;

//For smoothing the throttle
float Throttle_cmd;
float smt_Throttle;
bool prev_dir = 0;
int brk_delay = 500;
unsigned long brk_timer = 0;

//Servo Object defination
Servo steeringServo;
Servo ESC ;
Servo gearServo;
Servo frontTLockServo;
Servo rearTLockServo;
Servo cameraServo;

//rpmVal_data Publisher for sensor readings
std_msgs::Int16MultiArray rpmVal;
ros::Publisher rpmVal_data("rpmVal_data", &rpmVal);

//optiFlow_data Publisher for sensor readings
std_msgs::Int16MultiArray optiFlow;
ros::Publisher optiFlow_data("optiFlow_data", &optiFlow);

//Failsafe output
void failsafe_out(){
  ESC.writeMicroseconds(1500);
  steeringServo.writeMicroseconds(1850);
}

// Arduino 'map' funtion for floating point
float fmap (float toMap, float in_min, float in_max, float out_min, float out_max) {
  return (toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float fw_to_rev(float th){
  if (brk_timer == 0){
    brk_timer = millis();
  }

  if((millis() - brk_timer) < brk_delay ){
    return th;
  }
  else{

    if((millis() - brk_timer) < (brk_delay + 100)){
      return 1500.0;
    }
    else{
      brk_timer = 0;
      prev_dir = 1;
      return th;
    }

  }
}

float rev_to_fw(){
  if (brk_timer == 0){
    brk_timer = millis();
  }

  if((millis() - brk_timer) < (brk_delay + 100) ){
    return 1500;
  }
  else{
      brk_timer = 0;
      prev_dir = 0;
      return 1500;
  }

}

void driveCallback ( const std_msgs::Float32MultiArray&  control_msg )
{
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
  steeringServo.writeMicroseconds(steer_cmd);


  if ((control_msg.data[1] == 0)){
    Throttle_cmd = 1500;
    smt_Throttle = 1500;
    ESC.writeMicroseconds(1500);
  }
  // ESC forward continue
  if ((control_msg.data[1] >= 0) && (prev_dir == 0)){
    Throttle_cmd = (float)fmap(control_msg.data[1], 0.0 , 1.0, 1500.0, maxThrottle);
    if(Throttle_cmd > maxThrottle){
      Throttle_cmd = maxThrottle;
    }
    smt_Throttle = smt_Throttle + 0.2 * (Throttle_cmd - smt_Throttle);
    ESC.writeMicroseconds(smt_Throttle);
  }
  
  //ESC reverse continue 
  if ((control_msg.data[1] < 0 ) && (prev_dir == 1)){
    Throttle_cmd = (float)fmap(control_msg.data[1], -1.0 , 0.0, minThrottle, 1500);
    if(Throttle_cmd < minThrottle){
      Throttle_cmd = minThrottle;
    }
    smt_Throttle = smt_Throttle + 0.2 * (Throttle_cmd - smt_Throttle);
    ESC.writeMicroseconds(smt_Throttle);
  }

  //From forward to rev
  if ((control_msg.data[1] < 0 ) && (prev_dir == 0)){
    
    Throttle_cmd = fw_to_rev(control_msg.data[1]);
    if(Throttle_cmd < minThrottle){
      Throttle_cmd = minThrottle;
    }
    smt_Throttle = 1500;
    ESC.writeMicroseconds(Throttle_cmd);
  }

  //From rev to forward
  if ((control_msg.data[1] > 0 ) && (prev_dir == 1)){
    
    Throttle_cmd = rev_to_fw();
    smt_Throttle = 1500;
    ESC.writeMicroseconds(Throttle_cmd);
  }

  /*
  if (control_msg.data[2] == 0.0) {
    Throttle_cmd = (float)fmap(control_msg.data[1], 0.5, 1.0, 1500.0, maxThrottle);
  } else {
    Throttle_cmd = (float)fmap(control_msg.data[1], 0.5, 1.0, 1500.0, minThrottle);
  }
  if (Throttle_cmd < minThrottle) {
    Throttle_cmd = minThrottle;
  }
  if (Throttle_cmd > maxThrottle) {
    Throttle_cmd = maxThrottle ;
  }
  
  if (digitalRead(38)) {        //reads signal from RF Reciever. Will set throttle to neutral if signal is high
    delay(5);
    if (digitalRead(38)) {
      electronicSpeedController.writeMicroseconds(1500) ;
    }
  }
  else if (control_msg.data[2] == 1.0) {
    //electronicSpeedController.writeMicroseconds(1400); //brakes
    electronicSpeedController.writeMicroseconds(Throttle_cmd) ; 
  }
  else {
    escThrottle = escThrottle + 0.05 * (Throttle_cmd - escThrottle); //Exponential Smoothing ( ͡° ͜ʖ﻿ ͡°) for throttle
    electronicSpeedController.writeMicroseconds(Throttle_cmd) ;
  }
  */

  //Handle for Gear command
  float gearPose = fmap(control_msg.data[3], 0.0, 1.0, servoLow, servoHigh);
  if (gearPose < servoLow) {
    gearPose = servoLow;
  }
  if (gearPose > servoHigh) {
    gearPose = servoHigh ;
  }
  gearServo.writeMicroseconds(gearPose);

  //Handle for Front Differential Lock command
  float fTLockPose = fmap(control_msg.data[4], 0.0, 1.0, servoLow, servoHigh);
  if (fTLockPose < servoLow) {
    fTLockPose = servoLow;
  }
  if (fTLockPose > servoHigh) {
    fTLockPose = servoHigh ;
  }
  frontTLockServo.writeMicroseconds(fTLockPose);

  //Handle for Rear Differential Lock command
  float rTLockPose = fmap(control_msg.data[5], 0.0, 1.0, servoLow, servoHigh);
  if (rTLockPose < servoLow) {
    rTLockPose = servoLow;
  }
  if (rTLockPose > servoHigh) {
    rTLockPose = servoHigh ;
  }  
  rearTLockServo.writeMicroseconds(rTLockPose);

  //Handle for Camera angle command
  float cameraAngle = fmap(control_msg.data[6], 0.0, 1.0, cameraLow, cameraHigh);
  if (cameraAngle < cameraLow) {
    cameraAngle = cameraLow;
  }
  if (cameraAngle > cameraHigh) {
    cameraAngle = cameraHigh ;
  } 
  cameraServo.writeMicroseconds(cameraAngle);
  
}

ros::Subscriber<std_msgs::Float32MultiArray> driveSubscriber("/cmd_vel1", &driveCallback);

void setup() {
  
  Serial.begin(57600);

  //ROS initialization
  nodeHandle.initNode();
  nodeHandle.subscribe(driveSubscriber);
  nodeHandle.advertise(rpmVal_data);
  nodeHandle.advertise(optiFlow_data);

  //Initilize interrupt for RPM sensor
  DDRK = 0;
  for(uint8_t i = 0; i < PCINT_PIN_COUNT; i++){ // i think a for loop is ok for the init.
      PCINT_RX_PORT |= PCInt_RX_Pins[i];
      PCINT_RX_MASK |= PCInt_RX_Pins[i];
    }
  PCICR = PCIR_PORT_BIT;

  // Attach the servos to actual pins
  steeringServo.attach(st_svr);
  ESC.attach(esc); 
  gearServo.attach(g_svr);
  frontTLockServo.attach(fl_svr);
  rearTLockServo.attach(rl_svr);
  cameraServo.attach(cm_svr);

  // Initialize Servos and ESC setting
  // Steering centered is 90, throttle at neutral is 90 ,Lo gear is 0, front differential lock is 0, rear differential lock is 0, Camera angle is 0
  steeringServo.writeMicroseconds(1500);
  ESC.writeMicroseconds(1500);
  gearServo.writeMicroseconds(1800);
  frontTLockServo.writeMicroseconds(1800);
  rearTLockServo.writeMicroseconds(1200);
  cameraServo.writeMicroseconds(cameraLow);
  delay(100);

  //PM3901 flow sensor initialize
  for(int i=0;i<10;i++){
    if (flow.begin()){
      flowVal[3] = 0;
      //Serial.println("initialization complete ");
      break;
    }
    //Serial.print("initialization attempt: ");
    //Serial.println(i);
    delay(100);
  }
  

  //VL53L1x Sensor initialize
  Wire.begin();
  Wire.setClock(100000); // use 400 kHz I2C
  sensor.setTimeout(500);
  for(int i=0;i<10;i++){
    if (sensor.init()){
      flowVal[4] = 0;
      sensor.setDistanceMode(VL53L1X::Short);
      sensor.setMeasurementTimingBudget(20000);
      sensor.startContinuous(5);
      //Serial.println("initialization complete ");
      break;
    }
    //Serial.print("initialization attempt: ");
    //Serial.println(i);
    delay(100);
  }

  
}

#define RX_PIN_CHECK(pin_pos, rc_value_pos) \
  if (mask & PCInt_RX_Pins[pin_pos]) {      \                      
    if (!(pin & PCInt_RX_Pins[pin_pos])) {  \
      if ((cTime-edgeTime[pin_pos]) > 10){ \                     
        rpmValue[pin_pos] = rpmValue[pin_pos] + 1; \
      }\                            
    } else edgeTime[pin_pos] = cTime; \                           
   }

#define RX_PIN_CHECK(pin_pos, rc_value_pos) \
  if (mask & PCInt_RX_Pins[pin_pos]) {      \                      
    if (!(pin & PCInt_RX_Pins[pin_pos])) {  \
      if ((cTime-edgeTime[pin_pos]) > 10){ \                     
        rpmValue[pin_pos] = rpmValue[pin_pos] + 1; \
      }\                            
    } else edgeTime[pin_pos] = cTime; \                           
   }

ISR(RX_PC_INTERRUPT) { //this ISR is common to every receiver channel, it is call everytime a change state occurs on a RX input pin
    uint8_t mask;
    uint8_t pin;
    uint16_t cTime,dTime;
    static uint16_t edgeTime[8];
    static uint8_t PCintLast;
  
    pin = RX_PCINT_PIN_PORT; // RX_PCINT_PIN_PORT indicates the state of each PIN for the arduino port dealing with Ports digital pins
   
    mask = pin^PCintLast;   // doing a ^ between the current interruption and the last one indicates wich pin changed
    cTime = micros();         // micros() return a uint32_t, but it is not usefull to keep the whole bits => we keep only 16 bits
    sei();                    // re enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
    PCintLast = pin;          // we memorize the current state of all PINs [D0-D7]
  
    #if (PCINT_PIN_COUNT > 0)
      RX_PIN_CHECK(0,0);
    #endif
    #if (PCINT_PIN_COUNT > 1)
      RX_PIN_CHECK(1,1);
    #endif
    #if (PCINT_PIN_COUNT > 2)
      RX_PIN_CHECK(2,2);
    #endif
    #if (PCINT_PIN_COUNT > 3)
      RX_PIN_CHECK(3,3);
    #endif
    #if (PCINT_PIN_COUNT > 4)
      RX_PIN_CHECK(4,4);
    #endif
    #if (PCINT_PIN_COUNT > 5)
      RX_PIN_CHECK(5,5);
    #endif
    #if (PCINT_PIN_COUNT > 6)
      RX_PIN_CHECK(6,6);
    #endif
    #if (PCINT_PIN_COUNT > 7)
      RX_PIN_CHECK(7,7);
    #endif
} 


void loop()
{
  unsigned long currentMillis = millis();
  rpmVal.data_length = 4;
  optiFlow.data_length = 5;
  if((millis() - last_msg_time) > 1000){
    failsafe_out();
  } 
  
  //RPM sensor data loop running at 2Hz
  if (currentMillis - previousMillis1 >= 500){
    for (int i = 0; i < PCINT_PIN_COUNT; i++ ){
      /*
      Serial.print("S");
      Serial.print(i);
      Serial.print(": ");
      Serial.print(rpmValue[i]);
      Serial.print("\t");
      */
      rpm[i] = (rpmValue[i])*2*60/8;
      rpmValue[i] = 0;    
    }
    //Serial.println();
    rpmVal.data = rpm;
    rpmVal_data.publish( &rpmVal);
    previousMillis1 = currentMillis;
  }

  //OptiFlow sensor data loop running at 50Hz
  if (currentMillis - previousMillis2 >= 20){
    
    //Read X and Y displacement if sensor initialized
    if(flowVal[3] == 0){
      flow.readMotionCount(&deltaX, &deltaY);
      flowVal[0] = deltaY;
      flowVal[1] = deltaX;
    }

    //Read Z distance if sensor initialized
    if(flowVal[4] == 0){
      distZ = sensor.read(false);
      if((distZ != 0) || (flsf1 > 10)){
        flowVal[2] = distZ;
        flsf1 = 0;
      }
      else{
        flsf1 = flsf1 + 1;
      }
   

      if (sensor.timeoutOccurred())
      {
        flsf2 = flsf2 + 1;
      }
      else{
        flsf2 = 0;
      }
  
      if(flsf2 >20){
        flowVal[4] = -1;
        flsf2 = 0;
      }
    }
    //Pack the data and publish
    optiFlow.data = flowVal;
    optiFlow_data.publish( &optiFlow);
    
    /*
    Serial.print("X: ");
    Serial.print(flowVal[0]);
    Serial.print("  Y: ");
    Serial.print(flowVal[1]);
    Serial.print("  Sens: ");
    Serial.println(flowVal[3]);
    */
    previousMillis2 = currentMillis;
  }
  
  nodeHandle.spinOnce();
  delay(1);
}
