#ifndef DEF_H_
#define DEF_H_

//status LED pin defination
#define conLed 28
#define armLed 29
#define modeLed 30
#define sLckLed 31

//servo pin defination
#define st_svr 3   //Steering Servo
#define esc 5      //Electronic Speed Controller
#define g_svr 7    //Hi-LO gear servo
#define fl_svr 9   //Front Differential Lock Servo
#define rl_svr 11  //Rear Differential Lock Servo
#define cm_svr 13  //Camera Tilt Servo

// Limits for the servo range
#define minSteering 1150 
#define maxSteering 1850 
#define minThrottle 1200
#define maxThrottle 1900
#define servoLow 1050
#define servoHigh 1850
#define cameraLow 700
#define cameraHigh 2200

#define max_speed 70
#define brk_delay 500

//RPM sensor reading pin and interrupt definations
#define PCINT_PIN_COUNT            6
#define PCINT_RX_BITS              (1<<0),(1<<1),(1<<2),(1<<3),(1<<5),(1<<7)
#define PCINT_RX_PORT              PORTK
#define PCINT_RX_MASK              PCMSK2
#define PCIR_PORT_BIT              (1<<2)
#define RX_PC_INTERRUPT            PCINT2_vect
#define RX_PCINT_PIN_PORT          PINK
#define wheelSensPins              {0, 1, 2, 3}
#define motorSensPins              {4, 5}

#define kp 2.35
#define ki 0.21
#define kd 0.32

#define pidTimer 20
#define rpmPubTimer 200
#define flowPubTimer 100

#endif
