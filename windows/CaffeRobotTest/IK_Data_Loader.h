#pragma once
#include "ARMSDK\include\ARMSDK.h"
#include <opencv2\opencv.hpp>
#include <conio.h>
#include <strsafe.h>
#include <Windows.h>

#define _USE_MATH_DEFINES
#include <math.h>

class IK_Data_Loader
{
public:
	IK_Data_Loader();
	~IK_Data_Loader();

	void LoadDataAll(char *RootPath);
	void getData(cv::Mat *rgb, cv::Mat *depth, cv::Mat *ang, cv::Mat *rgbori, float *dist, cv::Point3f *endeff, int idx);
	void FreeDataAll();
	void createEndeffector(char *RootPath, armsdk::Kinematics *kin);
	void writeAngle(int *src, int idx);

	int getCount();

private:
	typedef struct filepath_{
		char rgbpath[256];
		char depthpath[256];
		char angpath[256];
		char rgbOriPath[256];
		char newPath[256];
	}FilePath;

	int height_, width_;

	std::vector<FilePath> FileList;
};

