# ECE785
Advanced Computer Design: Embedded Linux

Project 1: Optimizing the Sppherical Geometry Code
---------------------------------------------------
Optimize the execution time of the Find_Nearest_Waypoint function in geometry.c on the Beaglebone Black Wireless
Optimization performed:
1) Reduced Pipeline Stalls
2) Cos Approximation
3) Math Optimizations (Avoiding floating point operations)
-Profiling using Perf

Starter Code- Average Execution time: 74 μsec 
Final Code- Average Execution time: 43 μsec

Project 2: Vectorizing th Spherical Geometry Code
--------------------------------------------------
Optimize the execution time of the Find_Nearest_Waypoint function in geometry.c using vectorization with NEON Advanced SIMD instructions on the Beaglebone Black Wireless.
(Neon SIMD operations via compiler intrinsics)

Optimization performed:
1) Vectorizing Cos Function
2) Vectorizing the array and radius comparisons
3) Vectorizing other calculations(max value and index search)
4) Inlining functions (Reducing function calls)

Starter Code- Average Execution time: 43 μsec 
Final Code- Average Execution time: 18μsec

Project 3: Image Processing and Stabilization using BeagleBone Black
---------------------------------------------------------------------
Interface webcam  with BeagleBone Black and use openCV to identify the location of two reference marks in an image, and calculate pan and tilt errors.

Steps:
1) Installing OpenCV on Beaglebone Black
2) Capturing (frames) Image using WebCam
3) Image procesing for item identification (Blurring, Detecting color codes, Detecting edges and contours,etc.)
4) Computation (Midpoint calculations, Line equation)
5) Pan and Tilt Error calculation
6) Generate Output Image
