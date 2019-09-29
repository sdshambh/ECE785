/*----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/
#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include "geometry.h"
#include "sincos.h"

#define TEST1_LAT (45.0)
#define TEST1_LON (79.0)

#define N_TESTS (10000)
extern const CT_T capitals[];

/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
	float dist, bearing, cur_pos_lat, cur_pos_lon;
	char * name;
	char capital_name[50];
	struct timespec start, end;
	uint64_t diff, total=0, min=1234567890;
	int n=0, m=0;
	
	while(strcmp(capitals[m].Name, "END"))
	{
		total=0;
		min=1234567890;
		cur_pos_lat = capitals[m].Lat;
		cur_pos_lon = capitals[m].Lon;
		//capital_name = capitals[m].Name;
		strcpy(capital_name, capitals[m].Name);
		 
		/*	printf("Current location is %f deg N, %f deg W\n", cur_pos_lat,
			cur_pos_lon);
		*/#if VALIDATION==0
	for (n=0; n<N_TESTS; n++)
		#endif	
		{
		clock_gettime(CLOCK_THREAD_CPUTIME_ID, &start);
		Find_Nearest_Waypoint(cur_pos_lat, cur_pos_lon,
					&dist, &bearing, &name);
		clock_gettime(CLOCK_THREAD_CPUTIME_ID, &end);
	
		diff = 1000000000 * (end.tv_sec - start.tv_sec) +
			end.tv_nsec - start.tv_nsec;
		//	  printf("%2d: %8lu ns\n", n, diff);
		total += diff;
		if (diff < min)
			min = diff;
		}
		printf("Closest waypoint to %s is %s. %f km away at bearing %f degrees\n",
			capital_name, name, dist, bearing);
	#if VALIDATION==0
		printf("Average %10.3f us\n", total/(1000.0*N_TESTS));
		printf("Minimum %10.3f us\n",  min/1000.0);
		#endif
#if VALIDATION==1
		printf("Average %10.3f us\n", total/(1000.0));
		printf("Minimum %10.3f us\n",  min/1000.0);
		#endif
		m++;
	}
	exit(0);
}

