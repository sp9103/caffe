#include "Pregrasp_Data_Loader.h"


Pregrasp_Data_Loader::Pregrasp_Data_Loader()
{
}


Pregrasp_Data_Loader::~Pregrasp_Data_Loader()
{
}

void Pregrasp_Data_Loader::LoadDataAll(char *RootPath){
	WIN32_FIND_DATA ffd;
	HANDLE hFind = INVALID_HANDLE_VALUE;
	TCHAR szDir[MAX_PATH] = { 0, };

	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, RootPath, strlen(RootPath), szDir, MAX_PATH);
	StringCchCat(szDir, MAX_PATH, TEXT("\\*"));

	hFind = FindFirstFile(szDir, &ffd);
	while (FindNextFile(hFind, &ffd) != 0){
		TCHAR subDir[MAX_PATH] = { 0, };
		memcpy(subDir, szDir, sizeof(TCHAR)*MAX_PATH);
		size_t len;
		StringCchLength(subDir, MAX_PATH, &len);
		subDir[len - 1] = '\0';
		StringCchCat(subDir, MAX_PATH, ffd.cFileName);
		char tBuf[MAX_PATH];
		WideCharToMultiByte(CP_ACP, 0, subDir, MAX_PATH, tBuf, MAX_PATH, NULL, NULL);

		//Tchar to char
		char ccFileName[256];
		WideCharToMultiByte(CP_ACP, 0, ffd.cFileName, len, ccFileName, 256, NULL, NULL);
		printf("directory : %s load.\n", ccFileName);

		if (ccFileName[0] != '.'){
			WIN32_FIND_DATA class_ffd;
			TCHAR szProcDir[MAX_PATH] = { 0, };
			HANDLE hDataFind = INVALID_HANDLE_VALUE;
			char procDir[256];
			strcpy(procDir, tBuf);
			strcat(procDir, "\\PROCESSIMG2\\*");
			MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, procDir, strlen(procDir), szProcDir, MAX_PATH);
			hDataFind = FindFirstFile(szProcDir, &class_ffd);

			while (FindNextFile(hDataFind, &class_ffd) != 0){
				//1. Image load
				char ProcFileName[256];
				size_t Proclen;
				StringCchLength(class_ffd.cFileName, MAX_PATH, &Proclen);
				WideCharToMultiByte(CP_ACP, 0, class_ffd.cFileName, 256, ProcFileName, 256, NULL, NULL);

				if (ProcFileName[0] == '.')
					continue;

				char DepthFile[256], rgbImgFile[256], ApproachPath[256];
				FILE *fp;
				int filePathLen;
				//image load
				strcpy(rgbImgFile, procDir);
				filePathLen = strlen(rgbImgFile);
				rgbImgFile[filePathLen - 1] = '\0';
				strcat(rgbImgFile, ProcFileName);

				//3.depth 읽어오기
				sprintf(DepthFile, "%s\\PROCDEPTH2\\%s", tBuf, ProcFileName);
				int depthwidth, depthheight, depthType;
				filePathLen = strlen(DepthFile);
				DepthFile[filePathLen - 1] = 'n';
				DepthFile[filePathLen - 2] = 'i';
				DepthFile[filePathLen - 3] = 'b';

				char AnglePath[256];
				TCHAR szAngleDir[MAX_PATH] = { 0, };
				sprintf(AnglePath, "%s\\ANGLE2\\%s", tBuf, ProcFileName);
				filePathLen = strlen(AnglePath);
				AnglePath[filePathLen - 1] = 't';
				AnglePath[filePathLen - 2] = 'x';
				AnglePath[filePathLen - 3] = 't';

				FilePath tempPath;
				strcpy(tempPath.angpath, AnglePath);
				strcpy(tempPath.depthpath, DepthFile);
				strcpy(tempPath.rgbpath, rgbImgFile);

				FileList.push_back(tempPath);
			}
		}
	}
}

void Pregrasp_Data_Loader::getData(cv::Mat *rgb, cv::Mat *depth, cv::Mat *rgbori, int idx){
	rgb->release();
	depth->release();
	rgbori->release();

	int height_, width_;
	height_ = 250;
	width_ = 400;

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

	int depthwidth, depthheight, depthType;
	FILE *fp = fopen(tempPath.depthpath, "rb");
	fread(&depthwidth, sizeof(int), 1, fp);
	fread(&depthheight, sizeof(int), 1, fp);
	fread(&depthType, sizeof(int), 1, fp);
	cv::Mat depthMap(depthwidth, depthheight, depthType);
	for (int i = 0; i < depthMap.rows * depthMap.cols; i++)		fread(&depthMap.at<float>(i), sizeof(float), 1, fp);
	fclose(fp);

	*rgb = tempdataMat.clone();
	*depth = depthMap.clone();
	*rgbori = img.clone();
}
void Pregrasp_Data_Loader::writeAngleData(float *ang, int idx){
	std::string path = FileList.at(idx).angpath;
	FILE *fp = fopen(path.c_str(), "w");
	for (int i = 0; i < 9; i++){
		fprintf(fp, "%d\n", (int)ang[i]);
	}
	fclose(fp);
	printf("%s\n", path.c_str());
}
void Pregrasp_Data_Loader::FreeDataAll(){
	FileList.clear();
}

int Pregrasp_Data_Loader::getCount(){
	return FileList.size();
}