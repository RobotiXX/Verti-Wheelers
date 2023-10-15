#ifndef SENSORS_H_
#define SENSORS_H_

int pm3901Init();
int VL53L1xInit();
void rpmInit();

void updateMotorRpm();
void updateWheelRpm();
void readDisplacement();
int readZHieght();

#endif

