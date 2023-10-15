#ifndef OUTPUT_H_
#define OUTPUT_H_

void initServos();
void writeServos();
void failSafeActive();
int throttlePID(int, int);

#endif