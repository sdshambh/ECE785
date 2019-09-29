#include "geometry.h"
#include <math.h>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include "sincos.h"
#define PI 3.14159265f
#define PI_OVER_180 (0.017453293f) 

extern const PT_T waypoints[]; 
//PT_T newarr[];

#define FAST 0
/*
11: radians in table, precalc'd sin, cos
12: Calc_Closeness
13: Don't do bearing
*/

// Table holds precalculated sin/cos for p2. Table Lat/Lon values are in radians



#if VALIDATION==1
double Calc_Bearing_double( PT_T * p1,  const PT_T * p2){
  // calculates bearing in degrees between locations (represented in radians)
  double term1, term2;
  double angle;

  term1 = sin(p1->Lon - p2->Lon)*p2->CosLat;
  term2 = p1->CosLat*p2->SinLat -
    p1->SinLat*p2->CosLat*cos(p1->Lon - p2->Lon);
  angle = atan2(term1, term2) * (180/PI);
  if (angle < 0.0)
    angle += 360;
  return angle;
}

double Calc_Closeness_double( PT_T * p1,  const PT_T * p2) { 
  // calculates closeness (decreases as distance increases) of locations

  return p1->SinLat * p2->SinLat +
    p1->CosLat * p2->CosLat*
    cos(p2->Lon - p1->Lon);
}



#endif
float Calc_Bearing( PT_T * p1,  const PT_T * p2){
  // calculates bearing in degrees between locations (represented in radians)
  float term1, term2;
  float angle;
  #if FAST ==0
	term1 = sinf(p1->Lon - p2->Lon)*p2->CosLat;
  term2 = p1->CosLat*p2->SinLat -
    p1->SinLat*p2->CosLat*cosf(p1->Lon - p2->Lon);
	#endif
	
	#if FAST==1
  term1 = sin_73(p1->Lon - p2->Lon)*p2->CosLat;
  term2 = p1->CosLat*p2->SinLat -
    p1->SinLat*p2->CosLat*cos_73(p1->Lon - p2->Lon);
  #endif
  
  angle = atan2f(term1, term2) * (180/PI);
  if (angle < 0.0)
    angle += 360;
  return angle;
}


float Calc_Closeness_fastcos( PT_T * p1,  const PT_T * p2) { 
  // calculates closeness (decreases as distance increases) of locations

  return p1->SinLat * p2->SinLat +
    p1->CosLat * p2->CosLat*
    cos_mycode(p2->Lon - p1->Lon);
}


float Calc_Closeness_cosapprox( PT_T * p1,  const PT_T * p2) { 
  // calculates closeness (decreases as distance increases) of locations

  return p1->SinLat * p2->SinLat +
    p1->CosLat * p2->CosLat*
    cos_73(p2->Lon - p1->Lon);
}

void Find_Nearest_Waypoint(float cur_pos_lat, float cur_pos_lon, float * distance, float * bearing, 
			   char  * * name) {
  // cur_pos_lat and cur_pos_lon are in degrees
  // distance is in kilometers
  // bearing is in degrees

  int i=0, closest_i=0;
  PT_T ref;
  int index_waypoint[200];
  float arr[200];
  float d, b, c, c1, max_c=0,closest_d=1E10,diff, diff1,diff2,radius,epsilon=0.011f;
  #if VALIDATION==1
  PT_T val;
  double d_double, b_double, c_double, max_c_double=0;
  int closest_i_double=0,valid=0;
  #endif
  struct timespec prestart, start, end1, end2;
  unsigned to;
typedef union{
	 float diff_float;
	 unsigned int  diff_int;
}unified;
 unified p,q,r;

//p.diff_float= &diff ;
//*(p.diff_int) = (unsigned int *)(p.diff_float);

//q.diff_float= &diff1 ;
//*(q.diff_int) = (unsigned int *)(q.diff_float);

//r.diff_float= &diff2 ;
//*(r.diff_int) = (unsigned int *)(r.diff_float);

  *distance = 0.0f;
  *bearing = 0;
  *name = '\0';

 

  ref.Lat = cur_pos_lat*PI/180;
  ref.Lon = cur_pos_lon*PI/180;
  ref.SinLat = sinf(ref.Lat);
  ref.CosLat = cosf(ref.Lat);
 #if VALIDATION==1
	val.Lat = cur_pos_lat*PI/180;
	val.Lon = cur_pos_lon*PI/180;
	val.SinLat = sin(val.Lat);
	val.CosLat = cos(val.Lat);
#endif

  strcpy(ref.Name, "Reference");

#if 0
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &prestart);
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
  
  to = start.tv_nsec - prestart.tv_nsec;
#endif
//    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
//unified.diff_float= &diff ;	
  while (strcmp(waypoints[i].Name, "END")) {
    c = Calc_Closeness_fastcos(&ref, &(waypoints[i]) );
(p.diff_float) = c - max_c;
arr[i]=c;
c1= Calc_Closeness_fastcos(&ref, &(waypoints[i+1]) );
(q.diff_float) = c1 - max_c;
arr[i+1]=c1;
 //   clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end1);
    if (!((p.diff_int) & 0x80000000)) {
      max_c = c;
      closest_i = i;
    }
	 if (!((q.diff_int) & 0x80000000)) {
      //max_c = c1;
      closest_i = i+1;
    }
	if(max_c<c1)
	{
		max_c=c1;
	}
	i=i+2;
 //   clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end2);

  }
//radius = epsilon + acosf(arr[closest_i])*6371;
//radius = 0.001 + arr[closest_i];
radius= max_c * ((100-epsilon)/(100+epsilon));
int k=0,e=0;
while(strcmp(waypoints[k].Name, "END"))
//for(i=0;i<163;i++)
{
	//newarr[i]=arr[closest_i]+
	if(arr[k]>=radius)
	{
		index_waypoint[e]=k;
		//printf("Closest waypoint is %s with c as %f\n", waypoints[k].Name,arr[k]);
		//memcpy(&newarr[k],&waypoints[i],sizeof(PT_T));
		//printf("Closest waypoint is %s\n", newarr[e].Name);
		e++;
	}
	k++;
}
//printf(" %d\n",e);
int t=0;
float g;
 max_c=0;
closest_i=0;
while(t<e)
{
	//*name = (char * ) (waypoints[closest_i].Name);
//printf("Closest waypoint is %s\n", newarr[t].Name);
g=Calc_Closeness_cosapprox(&ref, &(waypoints[index_waypoint[t]]));

if(g>max_c)
{
	max_c=g;
	closest_i=index_waypoint[t];
}
	t++;
}

/*
  printf("Start to End 1: %d\t", end1.tv_nsec - start.tv_nsec - to);
  printf("Start to End 2: %d\n", end2.tv_nsec - start.tv_nsec - to);
*/
  // Finish calcuations for the closest point




d = acosf(max_c)*6371; // finish distance calcuation
b = Calc_Bearing(&ref, &(waypoints[closest_i]) );
*distance = d;
  *bearing = 360-b;
  *name = (char * ) (waypoints[closest_i].Name);
#if VALIDATION==1 
while (strcmp(waypoints[valid].Name, "END")) {
    c_double = Calc_Closeness_double(&val, &(waypoints[valid]) );
 //   clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end1);
 
    if (c_double>max_c_double) {
      max_c_double = c_double;
      closest_i_double = valid;
    }
	valid++;
 //   clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end2);
}
  d_double = acos(max_c_double)*6371; // finish distance calcuation
  b_double = Calc_Bearing_double(&val, &(waypoints[closest_i_double]) );
  double Val_error = fabs(d_double-d)*100/d_double;
    double Val_bearing_error = fabs(b_double - b);
 b_double= 360 - b_double;
 b= 360-b;
    if( Val_error >= 0.01 )
      printf("\n Distance error : Value = %f. Actual Value = %f.  Calculated = %f\n", Val_error, d_double, d);

    if(Val_bearing_error >= 0.1)
      printf("\n Bearing error : Value = %f. Actual Value = %f.  Calculated = %f\n", Val_bearing_error,b_double , b);
#endif

  // return information to calling function about closest waypoint 
  

}
