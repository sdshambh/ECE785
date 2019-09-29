#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <iostream>
#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <ctime>
using namespace cv;
using namespace std; 

int DELAY_BLUR = 100;
int MAX_KERNEL_LENGTH = 31; 
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;

int low_H = 15, low_S = 65, low_V =175;
int high_H = 180, high_S = 255, high_V = 255;

float pi_val = 57.29577;

cv::Mat src; cv::Mat dst;
Mat dst_HSV, canny_output;
char window_name[] = "Smoothing Demo";

struct centerPoint
{
	double x;
	double y;
};

int main() 
{
	clock_t start;
	double duration;

	// Load the image file and check for success
	VideoCapture cap;
	start = clock();
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    if(!cap.open(0))
        return 0;
    for(;;)
    {
        Mat frame;
        cap >> frame;
        if( frame.empty() ) break; // end of video stream
        imwrite("input_img.jpg", frame);
        if( waitKey(30) >=0 ) break;
    }
    cv::Mat src = cv::imread("input_img.jpg", 1);
	if(!src.data) 
	{
		std::cout << "Unable to open the image file" << std::endl;
		return -1;
	} 
	
	for ( int i = 1; i < MAX_KERNEL_LENGTH; i = i + 2 )
    {
       cv::blur( src, dst, cv::Size( i, i ), cv::Point(-1,-1) );
    }
	cvtColor(dst, dst, COLOR_BGR2HSV);
	inRange(dst, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), dst_HSV);
	
	centerPoint imgCentre;
	imgCentre.x = (src.size().width)/2;
	imgCentre.y = (src.size().height)/2;
	
	// detect edges using canny
	Canny( dst_HSV, canny_output, 50, 150, 3 );
 
	// find contours
	findContours( canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

	//get contours and centroid of figures
	vector<Moments> mu(contours.size());
	vector<Point2f> mc(contours.size());
	for( int i = 0; i<contours.size(); i++ )
	{ 
		mu[i] = moments( contours[i], false ); 
		mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
	}
 
	Mat drawing(canny_output.size(), CV_8UC3, Scalar(255,255,255));
	for( int i = 0; i<contours.size(); i++ )
	{
		Scalar color = Scalar(4,255,248); // B G R values
		drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point());
		circle( drawing, mc[i], 4, color, -1, 8, 0 );
	}
	
	centerPoint points[2];
	int j = 0;
	for( int i = 0; i<contours.size(); i+=2 )
	{ 
		points[j].x = (mc[i].x + mc[i+1].x)/2;
		points[j].y = (mc[i].y + mc[i+1].y)/2;
		j++;
	}
	
	centerPoint midpoint, joint;
	midpoint.x = (points[0].x + points[1].x)/2;
	midpoint.y = (points[0].y + points[1].y)/2;
	
	joint.x = midpoint.x;
	joint.y = imgCentre.y;
	
	//PAN and TILT error
	double pan_diff = pow((imgCentre.x - joint.x),2) + pow((imgCentre.y - joint.y),2);
	double pan_error = sqrt(pan_diff);

	double tilt_diff = pow((midpoint.x - joint.x),2) + pow((midpoint.y - joint.y),2);
	double tilt_error = sqrt(tilt_diff);
	
	cout << "Pan Error: " << pan_error << endl;
	cout << "Tilt Error: " << tilt_error << endl;
	
	//ROLL error
	float m2 = (points[1].y - points[0].y) / (points[1].x - points[0].x);
	float m1 = 0.0;
	float m = (m2 - m1)/(1 + m1*m2);
	
	float theta = atan(m2)*pi_val;
	if(theta < 0)
	{
		theta = 90 + theta;
	}
	cout << "Roll Error (in degrees): " << theta << endl;
	
	duration = (clock() - start ) / (double) (CLOCKS_PER_SEC);

	line( drawing, Point( points[0].x, points[0].y ), Point( points[1].x, points[1].y), Scalar( 4, 255, 248 ),  2, 8 );
	
	vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );
    vector<Point2f>centers( contours.size() );
    vector<float>radius( contours.size() );
	
	for( int i = 0; i<contours.size(); i++ )
	{ 
		approxPolyDP( contours[i], contours_poly[i], 3, true );
		minEnclosingCircle( contours_poly[i], centers[i], radius[i] );
		circle( src, centers[i], (int)radius[i], Scalar( 255, 0, 0 ), 2 );
	}
	line( src, Point( points[0].x, points[0].y ), Point( points[1].x, points[1].y), Scalar( 255, 0, 0 ),  2, 8 );
	
	cv::imwrite("output_image.jpg", src);
	cout <<"Program-Run Time: "<< duration <<'\n';
	return 0;
}