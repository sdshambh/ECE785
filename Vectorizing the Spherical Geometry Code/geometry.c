#include "geometry.h"
#include <math.h>
#include <string.h>
#include <time.h>
#include <stdio.h>
#include <arm_neon.h>
#include <arm_acle.h>
#include "sincos.h"
#define PI 3.14159265f
#define PI_OVER_180 (0.017453293f) 

extern const PT_T waypoints[];
extern const float Lat[]; 
extern const float Lon[];
extern const float SinLat[];
extern const float CosLat[];
//PT_T newarr[];
// Math constants we'll use
#define DP_PI (3.1415926535897932384626433)	// pi
double const twopi_g=2.0*DP_PI;			// pi times 2
double const two_over_pi_g= 2.0/DP_PI;		// 2/pi

#define FAST 0
/*
11: radians in table, precalc'd sin, cos
12: Calc_Closeness
13: Don't do bearing
*/

// Table holds precalculated sin/cos for p2. Table Lat/Lon values are in radians

//function to print vectors with float data type
void print_float32x4(float32x4_t v4) {
  int i;
  float v[4];
  float32x2_t v2;

  v2 = vget_low_f32(v4);
  v[0] = vget_lane_f32(v2, 0);
  v[1] = vget_lane_f32(v2, 1);
  v2 = vget_high_f32(v4);
  v[2] = vget_lane_f32(v2, 0);
  v[3] = vget_lane_f32(v2, 1);
  
  for (i=0; i<4; i++)
    printf("%f \t", v[i]);
  
}

// function to print vectors with uint data type
void print_uint32x4(uint32x4_t v4) {
  int i;
  uint32_t v[4];
  uint32x2_t v2;

  v2 = vget_low_u32(v4);
  v[0] = vget_lane_u32(v2, 0);
  v[1] = vget_lane_u32(v2, 1);
  v2 = vget_high_u32(v4);
  v[2] = vget_lane_u32(v2, 0);
  v[3] = vget_lane_u32(v2, 1);
  
  for (i=0; i<4; i++)
    printf("%u \t", v[i]);
  
}

#if VALIDATION==1
float Calc_Bearing_float( PT_T * p1,  const PT_T * p2){
  // calculates bearing in degrees between locations (represented in radians)
  float term1, term2;
  float angle;

  term1 = sin(p1->Lon - p2->Lon)*p2->CosLat;
  term2 = p1->CosLat*p2->SinLat -
    p1->SinLat*p2->CosLat*cosf(p1->Lon - p2->Lon);
  angle = atan2(term1, term2) * (180/PI);
  if (angle < 0.0)
    angle += 360;
  return angle;
}

float Calc_Closeness_float( PT_T * p1,  const PT_T * p2) { 
  // calculates closeness (decreases as distance increases) of locations

  return p1->SinLat * p2->SinLat +
    p1->CosLat * p2->CosLat*
    cosf(p2->Lon - p1->Lon);
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


/* float Calc_Closeness_fastcos( PT_T * p1,  const PT_T * p2) { 
  // calculates closeness (decreases as distance increases) of locations

  return p1->SinLat * p2->SinLat +
    p1->CosLat * p2->CosLat*
    cos_mycode(p2->Lon - p1->Lon);
} */


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
  //int index_waypoint[200];
  //float arr[200];
  int32_t w[4];
  //uint32_t w_k[4];
  float d, b,/* c, c1,c2,c3,*/ max_c=0/*,closest_d=1E10diff, diff1,diff2,radius,epsilon=0.011f*/;
  float32x4_t temp,temp1,temp2,temp3;
  //float32x4x41_t temp_array;
  //float32x2_t v2,v3,v4;
  int32x2_t w1;
  //uint32_t w1_k;
  float32x4_t c_max_vector = vdupq_n_f32(0);
   float32x4_t c_max_vector_pre= vdupq_n_f32(0);
   int32_t a[]={0,0,0,0};
   int32x4_t max_value_indexes=vld1q_s32(&a);
   
  #if VALIDATION==1
  PT_T val;
  float d_float, b_float, c_float, max_c_float=0;
  
  int closest_i_float=0,valid=0;
  #endif
  struct timespec prestart, start, end1, end2;
  unsigned to;
/*typedef union{
	 float diff_float;
	 unsigned int  diff_int;
}unified;
 unified p,q,r,s;
*/

  *distance = 0.0f;
  *bearing = 0;
  *name = '\0';

 float32x4_t Lon_float,SinLat_float,CosLat_float;

  ref.Lat = cur_pos_lat*PI/180;
  //Lat_float=vdupq_n_f32(ref.Lat);
  ref.Lon = cur_pos_lon*PI/180;
  Lon_float=vdupq_n_f32(ref.Lon);
  ref.SinLat = sinf(ref.Lat);
  SinLat_float=vdupq_n_f32(ref.SinLat);
  ref.CosLat = cosf(ref.Lat);
  CosLat_float=vdupq_n_f32(ref.CosLat);
 #if VALIDATION==1
	val.Lat = cur_pos_lat*PI/180;
	val.Lon = cur_pos_lon*PI/180;
	val.SinLat = sin(val.Lat);
	val.CosLat = cosf(val.Lat);
#endif

  strcpy(ref.Name, "Reference");

#if 0
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &prestart);
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
  
  to = start.tv_nsec - prestart.tv_nsec;
#endif
//    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
//unified.diff_float= &diff ;	
  //while (strcmp(waypoints[i].Name, "END")) {
	  
// vectorizing cos function,index,max value
	  float32x4_t SinLat_v,CosLat_v,Lon_v;
	  for(i=0;i<164;i+=4){
//finding vectors of SinLat,CosLat,Lon
	SinLat_v =vld1q_f32(&(SinLat[i]));
	CosLat_v =vld1q_f32(&(CosLat[i]));
	Lon_v =vld1q_f32(&(Lon[i]));
	//float32x4_t SinLat =vld1q_f32(&(SinLat[i]));
	//temp=SinLat_v;
	//temp=CosLat_v;
	
	// finding vector multiplication for fast cos function
	temp1=vmulq_f32( CosLat_float , CosLat_v);
	temp3=vsubq_f32 (Lon_v , Lon_float);
	int32x4_t quad = vcvtq_s32_f32(vmulq_f32(temp3,vdupq_n_f32(two_over_pi_g)));
	float32_t a=0.9940307;
	float32_t b=-0.4855807;
	float32x4_t c1=vld1q_dup_f32(&a);
	float32x4_t c2=vld1q_dup_f32(&b);
	float32x4_t case0,case1,case2,case3,cosine_result;
	uint32x4_t e0,e2,e3,e1;
	
	// calculation of c1+c2 *(x^2)
	case0= vaddq_f32(c1,vmulq_f32(vmulq_f32(temp3,temp3),c2));
	case1= vaddq_f32(c1,vmulq_f32(vmulq_f32((vsubq_f32(vdupq_n_f32(DP_PI),temp3)),(vsubq_f32(vdupq_n_f32(DP_PI),temp3))),c2));
	case2= vaddq_f32(c1,vmulq_f32(vmulq_f32((vsubq_f32(temp3,vdupq_n_f32(DP_PI))),(vsubq_f32(temp3,vdupq_n_f32(DP_PI)))),c2));
	case3= vaddq_f32(c1,vmulq_f32(vmulq_f32((vsubq_f32(vdupq_n_f32(twopi_g),temp3)),(vsubq_f32(vdupq_n_f32(twopi_g),temp3))),c2));

	// negating cos values for 2nd and 3rd quadrant
	case1= vnegq_f32(case1);
	case2= vnegq_f32(case2);
	
	//quants vectors initialization for check
	int32x4_t zero=vdupq_n_s32(0);
	int32x4_t one=vdupq_n_s32(1);
	int32x4_t two=vdupq_n_s32(2);
	int32x4_t three=vdupq_n_s32(3);
	
	// get the quadrants of all 4 elements
	e0=vceqq_s32(quad,zero);
	e1=vceqq_s32(quad,one);
	e2=vceqq_s32(quad,two);
	e3=vceqq_s32(quad,three);
	
	//cos value for each element according to their quadrants
	cosine_result=vbslq_f32(e0,case0,vdupq_n_f32(0.0));
	cosine_result= vbslq_f32(e1,case1,cosine_result);
	cosine_result= vbslq_f32(e2,case2,cosine_result);
	cosine_result=vbslq_f32(e3,case3,cosine_result);

	//float32x4_t result=vaddq_t(element1,element2);
	//return cosine_result;
	
	temp2=cosine_result;
	temp=vaddq_f32((vmulq_f32(SinLat_float , SinLat_v)) , vmulq_f32(temp1 ,temp2 ));
	
	//get cos value in each lane and put it in an array
	/*c= vget_lane_f32(temp,0);
	c1=vget_lane_f32(temp,1);
	c2=vget_lane_f32(temp,2);
	c3=vget_lane_f32(temp,3); 
	*/

   //v2 = vget_low_f32(temp);
  //c = vget_lane_f32(v2, 0);
  //c1 = vget_lane_f32(v2, 1);
  //v2 = vget_high_f32(temp);
  //c2 = vget_lane_f32(v2, 0);
  //c3 = vget_lane_f32(v2, 1); 
	//arr[i]= c;
	//arr[i+1]=c1;
	//arr[i+2]=c2;
	//arr[i+3]=c3;

	//i=i+2;
 //   clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end2);
//find max of all elements and set max_c to calculate radius
 /*for(int j=1;j<4;j++)
{
	if(max_c<arr[i+j])
	max_c=arr[i+j];
}*/

//vectorize c_max value calculation
//update the indexes for c_max value (selecting based on previous and new value)
//update if value has changed else take previous value
int array[]={i+0,i+1,i+2,i+3};
int32x4_t array_vector=vld1q_s32(&array[0]);
c_max_vector=vmaxq_f32(temp,c_max_vector);
uint32x4_t ene=vceqq_f32(c_max_vector_pre,c_max_vector);
//get indexes
max_value_indexes=vbslq_s32(ene,max_value_indexes,array_vector);
c_max_vector_pre=c_max_vector;

  }
 // printf("max c value is %f \n",max_c);
//radius = epsilon + acosf(arr[closest_i])*6371;
//radius = 0.001 + arr[closest_i];
//get all the indexes from each lane
	w1=vget_low_s32(max_value_indexes);
	w[0]=vget_lane_s32(w1, 0);
	w[1]=vget_lane_s32(w1, 1);
	w1=vget_high_s32(max_value_indexes);
	w[2]=vget_lane_s32(w1, 0);
	w[3]=vget_lane_s32(w1, 1);
	//printf("The values in w is %u %u %u %u",w[0],w[1],w[2],w[3]);
	
	/*	for(int j=1;j<4;j++)
{
	if(max_c<w[j])
	max_c=w[j];
}
radius= max_c * ((100-epsilon)/(100+epsilon));
*/

	//vectorizing radius
//float32_t radius_v=radius;
//int k=0,e=0;
//float32x4_t temp_radius=(vdupq_n_f32(radius));
//float32x4_t temp_arr;
//uint32x4_t temp_status;
//while(strcmp(waypoints[k].Name, "END"))
	
	/*for(k=0;k<164;k+=4)
	{
	temp_arr=vld1q_f32(&(w[0]));
	//print_float32x4(temp_arr);
	//printf("\n");
	temp_status=vcgeq_f32(temp_arr,temp_radius);
		//print_uint32x4(temp_status);
	//printf("\n");
	w1_k=vget_low_s32(temp_status);
	w_k[0]=vget_lane_u32(w1_k, 0);
	w_k[1]=vget_lane_u32(w1_k, 1);
	w1_k=vget_high_s32(temp_status);
	w_k[2]=vget_lane_u32(w1_k, 0);
	w_k[3]=vget_lane_u32(w1_k, 1);
	for(int p=0;p<4;p++)
	{

		if(w[p]!=0)
		{
			
			index_waypoint[e]=k+p;
			e++;
		}
	}
	*/
//}
/*for(int l=0;l<e;l++)
{
printf("index %d \n",index_waypoint[l]);
}*/
//printf(" %d\n",e);
	int t=0;
	float g;
	 max_c=0;
	closest_i=0;
	//loop through all the indexes found and calculate max_c
	while(t<4)
	{
		//*name = (char * ) (waypoints[closest_i].Name);
	//printf("Closest waypoint is %s\n", newarr[t].Name);
	g=Calc_Closeness_cosapprox(&ref, &(waypoints[w[t]]));

	if(g>max_c)
	{
		max_c=g;
		closest_i=w[t];
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
		c_float = Calc_Closeness_float(&val, &(waypoints[valid]) );
	 //   clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end1);
	 
		if (c_float>max_c_float) {
		  max_c_float = c_float;
		  closest_i_float = valid;
		}
		valid++;
	 //   clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end2);
	}
	  d_float = acos(max_c_float)*6371; // finish distance calcuation
	  b_float = Calc_Bearing_float(&val, &(waypoints[closest_i_float]) );
	  float Val_error = fabs(d_float-d)*100/d_float;
		float Val_bearing_error = fabs(b_float - b);
	 b_float= 360 - b_float;
	 b= 360-b;
		if( Val_error >= 0.01 )
		  printf("\n Distance error : Value = %f. Actual Value = %f.  Calculated = %f\n", Val_error, d_float, d);

		if(Val_bearing_error >= 0.1)
		  printf("\n Bearing error : Value = %f. Actual Value = %f.  Calculated = %f\n", Val_bearing_error,b_float , b);
	#endif

	  // return information to calling function about closest waypoint 
	  

	}
