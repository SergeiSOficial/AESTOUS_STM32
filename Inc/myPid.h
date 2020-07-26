#ifndef MY_PID_H
#define MY_PID_H

float NTC_read(unsigned char termPosition);
char PidBigBlock(float SetTemp);
void freqPower(int power);

#endif //MY_PID_H
