#pragma once
#include <opencv2\opencv.hpp>
#include <conio.h>
#include <strsafe.h>
#include <Windows.h>

#define _USE_MATH_DEFINES
#include <math.h>

class Pregrasp_Data_Loader
{
public:
	Pregrasp_Data_Loader();
	~Pregrasp_Data_Loader();

	void LoadDataAll(char *RootPath);
	void getData(cv::Mat *rgb, cv::Mat *depth, cv::Mat *rgbori, int idx);
	void writeAngleData(float *ang, int idx);
	void FreeDataAll();

	int getCount();

private:
	typedef struct filepath_{
		char rgbpath[256];
		char depthpath[256];
		char angpath[256];
	}FilePath;

	std::vector<FilePath> FileList;
};

