#ifndef SINCOS_H
#define SINCOS_H
// geometry 
#include <arm_neon.h>
//float cos_mycode(float x);
//float cos_mycodes(float x);
 float cos_32s(float x);
 float cos_32(float x);
 float sin_32(float x);
 float cos_52s(float x);
 float cos_52(float x);
 float sin_52(float x);
 double cos_73s(double x);
 double cos_73(double x);
 double sin_73(double x);
 double cos_121s(double x);
 double cos_121(double x);
 double sin_121(double x);
 float tan_32(float x);
 float tan_32s(float x);
 double tan_82(double x);
 double tan_82s(double x);
double atan_66(double x);
double atan_66s(double x);
double atan_137(double x);
double atan_137s(double x);
float32x4_t cos_mycode(float32x4_t x);
#endif