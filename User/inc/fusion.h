#ifndef __FUSION_H
#define __FUSION_H


void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float *qt, float deltat_time);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz,  float *qt, float deltat_time);
#endif //__FUSION_H