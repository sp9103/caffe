#include "ListLoader.h"


ListLoader::ListLoader()
{
}


ListLoader::~ListLoader()
{
}

void ListLoader::LoadDataAll(char *RootPath){
	FILE *fp = fopen(RootPath, "r");
	FilePath tempPath;

	while (!feof(fp)){
		char imgPath[256], fileName[256], path[256];
		fscanf(fp, "%s", imgPath);
		tempPath.rgbpath = imgPath;
		int len = strlen(imgPath);
		int parseCount = 0;
		for (int i = len - 1; i > 0; i--){
			if (imgPath[i] == '\\'){
				parseCount++;
				imgPath[i] = '\0';

				if (parseCount == 1){
					memcpy(fileName, &imgPath[i + 1], sizeof(char) * (len - i));
				}
				else if (parseCount == 2){
					strcpy(path, imgPath);
				}
			}

			if (parseCount == 2)
				break;
		}

		//RGB Ori
		char AngDataFile[256], ProcImageFile[256], DepthFile[256], RgbOriFile[256];
		sprintf(RgbOriFile, "%s\\RGB\\%s", path, fileName);
		tempPath.rgbOriPath = RgbOriFile;

		//2. angle 읽어오기
		sprintf(AngDataFile, "%s\\ANGLE\\%s", path, fileName);
		len = strlen(AngDataFile);
		AngDataFile[len - 1] = 't';
		AngDataFile[len - 2] = 'x';
		AngDataFile[len - 3] = 't';
		tempPath.angpath = AngDataFile;

		//3.Depth 읽어오기
		sprintf(DepthFile, "%s\\DEPTHMAP2\\%s", path, fileName);
		len = strlen(DepthFile);
		DepthFile[len - 1] = 'n';
		DepthFile[len - 2] = 'i';
		DepthFile[len - 3] = 'b';
		tempPath.depthpath = DepthFile;

		FileList.push_back(tempPath);
	}

	fclose(fp);
}

void ListLoader::getData(cv::Mat *rgb, cv::Mat *depth, cv::Mat *ang, cv::Mat *rgbori, float *dist, cv::Point3f *endeff, int idx){
	rgb->release();
	depth->release();
	ang->release();
	rgbori->release();

	int height_, width_;
	height_ = width_ = 160;

	//angle min max
	int angle_max[9] = { 251000, 251000, 251000, 251000, 151875, 151875, 4095, 4095, 4095 };

	FilePath tempPath = FileList.at(idx);
	//rgb
	cv::Mat img = cv::imread(tempPath.rgbpath);
	cv::Mat tempdataMat(height_, width_, CV_32FC3);
	for (int h = 0; h < img.rows; h++){
		for (int w = 0; w < img.cols; w++){
			for (int c = 0; c < img.channels(); c++){
				tempdataMat.at<float>(c*height_*width_ + width_*h + w) = (float)img.at<cv::Vec3b>(h, w)[c] / 255.0f;
			}
		}
	}

	FILE *fp = fopen(tempPath.angpath.c_str(), "r");
	if (fp == NULL)		return;
	cv::Mat angMat(9, 1, CV_32FC1);
	cv::Mat angOriMat(9, 1, CV_32FC1);
	int angBox[9];
	bool angError = false;
	for (int i = 0; i < 9; i++){
		fscanf(fp, "%d", &angBox[i]);
		angMat.at<float>(i) = (float)angBox[i] / angle_max[i] * 180.f;
		angOriMat.at<float>(i) = (float)angBox[i];
		if (angBox[i] >= 250950 || angBox[i] <= -250950){
			angError = true;
			break;
		}
	}
	if (angError)		return;
	fclose(fp);

	int depthwidth, depthheight, depthType;
	fp = fopen(tempPath.depthpath.c_str(), "rb");
	fread(&depthwidth, sizeof(int), 1, fp);
	fread(&depthheight, sizeof(int), 1, fp);
	fread(&depthType, sizeof(int), 1, fp);
	cv::Mat depthMap(depthheight, depthwidth, depthType);
	for (int i = 0; i < depthMap.rows * depthMap.cols; i++)		fread(&depthMap.at<float>(i), sizeof(float), 1, fp);
	fclose(fp);

	*rgb = tempdataMat.clone();
	*depth = depthMap.clone();
	*ang = angOriMat.clone();
	*rgbori = img.clone();
}

void ListLoader::FreeDataAll(){
	FileList.clear();
}

int ListLoader::getCount(){
	return FileList.size();
}