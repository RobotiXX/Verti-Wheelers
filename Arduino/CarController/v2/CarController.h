#include "Arduino.h"
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>

extern int16_t deltaX, deltaY, distZ;
extern int16_t m_rpm[2], w_rpm[4];
extern int servo_values[6];
extern bool prev_dir;
extern unsigned long brk_timer;



