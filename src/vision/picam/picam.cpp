#include <stdio.h>
#include <unistd.h>
#include "camera.h"
#include <opencv2/opencv.hpp>
#include <chrono>	

#define WIDTH 640	
#define HEIGHT 480

unsigned char *dest;
static std::chrono::high_resolution_clock::time_point t1;
static std::chrono::high_resolution_clock::time_point t2;
static std::chrono::duration<double> time_span;
static double fps;

void startMilliSecondTimer(){
	t1 = std::chrono::high_resolution_clock::now();	
}

void CameraCallback(CCamera* cam, const void* buffer, int buffer_length)
{
	// check ellapsed time and increment fps as long as the the frame is received withthin the specified interval 
	t2 = std::chrono::high_resolution_clock::now();
        time_span = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);    	
	if(time_span.count() < 10 ){
		fps += 1;
	}else{
		printf("fps: %f\n", fps/time_span.count());
		startMilliSecondTimer();
		fps = 0;	
	}
	//printf("Ellapsed time: %.3f received %d bytes of data\n", time_span.count, buffer_length);
	cv::Mat myuv(HEIGHT + HEIGHT/2, WIDTH, CV_8UC1, (unsigned char*)buffer);
	cv::Mat mrgb(HEIGHT, WIDTH, CV_8UC4, dest);
	cv::cvtColor(myuv, mrgb, CV_YUV2RGBA_NV21); 
	//cv::imshow("video", mrgb);
	//cv::waitKey(1);
}

int main(int argc, const char **argv)
{
	printf("PI Cam api tester\n");
	dest = new unsigned char[HEIGHT*WIDTH*4];
	fps = 0;

	startMilliSecondTimer();
	StartCamera(WIDTH,HEIGHT,90,CameraCallback);
	sleep(30);
	StopCamera();
	delete[] dest;
}
