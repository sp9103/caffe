#pragma once
#include <opencv2\opencv.hpp>
#include <conio.h>
#include <strsafe.h>
#include <Windows.h>

#define _USE_MATH_DEFINES
#include <math.h>

class ListLoader
{
public:
	ListLoader();
	~ListLoader();

	void LoadDataAll(char *RootPath);
	void getData(cv::Mat *rgb, cv::Mat *depth, cv::Mat *ang, cv::Mat *rgbori, float *dist, cv::Point3f *endeff, int idx);
	void FreeDataAll();

	int getCount();

private:
	typedef struct filepath_{
		std::string rgbpath;
		std::string depthpath;
		std::string angpath;
		std::string rgbOriPath;
	}FilePath;

	std::vector<FilePath> FileList;
};

