#pragma once

//#ifndef WINDOW_HPP
//#define WINDOW_HPP
//#include <windows.h>
//#endif

#include <stdio.h>
#include <opencv2\opencv.hpp>
#include <time.h>
#include <math.h>

#define KINECT_COLOR_WIDTH		1920
#define KINECT_COLOR_HEIGHT		1080
#define KINECT_DEPTH_WIDTH		512
#define KINECT_DEPTH_HEIGHT		424

#define HEIGHT					250
#define WIDTH					400
#define CHANNEL					3
#define DATADIM					9

#define OPENCV_WAIT_DELAY		1

#define PI						3.141592653589

#define SWAP(a,b,t) ((t)=(a), (a)=(b), (b)=(t))
#define RIGHT_ARM_USE /*LEFT_ARM_USE*/
#define NUM_XEL			9

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}