#ifndef GEOMETRY_H
#define GEOMETRY_H
// geometry 
#include <arm_neon.h>
#include "config.h"
#define VALIDATION 0
void Find_Nearest_Waypoint(float cur_pos_lat, float cur_pos_lon, float * distance, float * bearing, 
			   char  * * name);
			   
typedef struct {
	float Lat;
	float SinLat;
	float CosLat;
	float Lon;
	char Name[24];
} PT_T;

typedef struct {
	float Lat;
	float Lon;
	char Name[50];
} CT_T;

#endif
