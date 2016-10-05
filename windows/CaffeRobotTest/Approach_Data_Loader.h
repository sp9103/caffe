#pragma once
#include <opencv2\opencv.hpp>
#include <conio.h>
#include <strsafe.h>
#include <Windows.h>

#define _USE_MATH_DEFINES
#include <math.h>

class Approach_Data_Loader
{
public:
	Approach_Data_Loader();
	~Approach_Data_Loader();

	void LoadDataAll(char *RootPath);
	void getData(cv::Mat *rgb, cv::Mat *depth, cv::Mat *rgbori, int idx);
	void writeAngleData(float *ang, int idx);
	void FreeDataAll();

	int getCount();
	
private:
	typedef struct filepath_{
		std::string rgbpath;
		std::string depthpath;
		std::string angpath;
	}FilePath;

	std::vector<FilePath> FileList;
};

