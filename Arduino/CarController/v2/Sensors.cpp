#include "Arduino.h"
#include <Bitcraze_PMW3901.h>
#include <Wire.h>
#include <VL53L1X.h>
#include "def.h"
#include "Sensors.h"
#include "CarController.h"


int wheelPins[] = wheelSensPins; 
int motorPins[] = motorSensPins;
Bitcraze_PMW3901 flow(53);
VL53L1X sensor;
static uint8_t PCInt_RX_Pins[PCINT_PIN_COUNT] = {PCINT_RX_BITS};
volatile uint16_t rpmValue[6] = {0,0,0,0,0,0};
int16_t deltaX = 0, deltaY = 0, distZ = 0;
int16_t m_rpm[2] = {0, 0};
int16_t w_rpm[4] = {0 ,0 ,0 ,0};

//PM3901 Opti-Flow sensor initialize
int pm3901Init(){
  deltaX = 0;
  deltaY = 0;
  for(int i=0;i<10;i++){
    if (flow.begin()){
     return 0;
    }
    //Serial.print("initialization attempt: ");
    //Serial.println(i);
    delay(50);
  }
  return 1;
}

//VL53L1 sensor Init 
int VL53L1xInit(){
  distZ = 0;
  Wire.begin();
  Wire.setClock(100000); // use 400 kHz I2C
  sensor.setTimeout(500);
  for(int i=0;i<10;i++){
    if (sensor.init()){
      sensor.setDistanceMode(VL53L1X::Short);
      sensor.setMeasurementTimingBudget(20000);
      sensor.startContinuous(5);
      return 0;
    }
    delay(100);
  }
  return 1;
}

//RPM sensor initialize
void rpmInit(){
  //Initilize interrupt for RPM sensor
  DDRK = 0;
  //assign interrupt to the pins
  for(uint8_t i = 0; i < PCINT_PIN_COUNT; i++){
      PCINT_RX_PORT |= PCInt_RX_Pins[i];
      PCINT_RX_MASK |= PCInt_RX_Pins[i];
    }
  PCICR = PCIR_PORT_BIT;
  m_rpm[0] = 0;
  m_rpm[1] = 0;
  w_rpm[0] = 0;
  w_rpm[1] = 0;
  w_rpm[2] = 0;
  w_rpm[3] = 0;
}

#define RX_PIN_CHECK(pin_pos, rc_value_pos) \
  if (mask & PCInt_RX_Pins[pin_pos]) {      \                      
    if (!(pin & PCInt_RX_Pins[pin_pos])) {  \
      if ((cTime-edgeTime[pin_pos]) > 1){ \                     
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
      RX_PIN_CHECK(wheelPins[0],0);
    #endif
    #if (PCINT_PIN_COUNT > 1)
      RX_PIN_CHECK(wheelPins[1],1);
    #endif
    #if (PCINT_PIN_COUNT > 2)
      RX_PIN_CHECK(wheelPins[2],2);
    #endif
    #if (PCINT_PIN_COUNT > 3)
      RX_PIN_CHECK(wheelPins[3],3);
    #endif
    #if (PCINT_PIN_COUNT > 4)
      RX_PIN_CHECK(motorPins[0],4);
    #endif
    #if (PCINT_PIN_COUNT > 5)
      RX_PIN_CHECK(motorPins[1],5);
    #endif
    #if (PCINT_PIN_COUNT > 6)
      RX_PIN_CHECK(6,6);
    #endif
    #if (PCINT_PIN_COUNT > 7)
      RX_PIN_CHECK(7,7);
    #endif
} 

//update motor rpm and clear the data
void updateMotorRpm(){
  m_rpm[0] = rpmValue[4];
  m_rpm[1] = rpmValue[5];
  rpmValue[4] = 0;
  rpmValue[5] = 0;
}

//update wheel rpm and clear the data
void updateWheelRpm(){
  w_rpm[0] = rpmValue[0];
  w_rpm[1] = rpmValue[1];
  w_rpm[2] = rpmValue[2];
  w_rpm[3] = rpmValue[3];
  rpmValue[0] = 0;
  rpmValue[1] = 0;
  rpmValue[2] = 0;
  rpmValue[3] = 0;
}

//update 2D displacement from flow deck sensor
void readDisplacement(){
  flow.readMotionCount(&deltaX, &deltaY);
}

//update z height from flowdeck sensor
int readZHieght(){
  distZ = sensor.read(false);
  if(distZ != 0){
    return 0;
  }
  if (sensor.timeoutOccurred()){
    return 1;
  }

}
