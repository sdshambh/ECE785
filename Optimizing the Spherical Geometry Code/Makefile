CC = gcc
CFLAGS = -c -Wall -march=armv7-a -mfpu=neon -mfloat-abi=hard -mcpu=cortex-a8 -mtune=cortex-a8 -O3 -g  -ffast-math -fsingle-precision-constant

sg: main.o geometry.o CMAN_coords.o sincos.o
	$(CC) main.o geometry.o CMAN_coords.o sincos.o -lrt -lm -g -static -o $@  

geometry_list.s: geometry.c
	$(CC) -Wa,-adhln -g geometry.c -c > geometry_list.s

clean:
	rm -f *.o sg *.s
