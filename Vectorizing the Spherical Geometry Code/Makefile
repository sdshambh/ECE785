CC = gcc
MORE_CFLAGS = -fno-tree-loop-vectorize -fno-tree-vectorize -fno-unroll-loops
CFLAGS = -c -Wall -march=armv7-a -mfpu=neon -mfloat-abi=hard -mcpu=cortex-a8 -mtune=cortex-a8 -O3 -g  -ffast-math -fsingle-precision-constant $(MORE_CFLAGS)
sg: main.o geometry.o CMAN_coords.o sincos.o
	$(CC) main.o geometry.o CMAN_coords.o sincos.o -lrt -lm -g -static -o $@  

geometry_list.s: geometry.c
	$(CC) -Wa,-adhln -g geometry.c -c > geometry_list.s

clean:
	rm -f *.o sg *.s
