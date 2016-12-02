#include "Approach_Data_Loader.h"


Approach_Data_Loader::Approach_Data_Loader()
{
}


Approach_Data_Loader::~Approach_Data_Loader()
{
}

void Approach_Data_Loader::LoadDataAll(char *RootPath){
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
			strcat(procDir, "\\RGB\\*");
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
				sprintf(DepthFile, "%s\\DEPTH\\%s", tBuf, ProcFileName);
				int depthwidth, depthheight, depthType;
				filePathLen = strlen(DepthFile);
				DepthFile[filePathLen - 1] = 'n';
				DepthFile[filePathLen - 2] = 'i';
				DepthFile[filePathLen - 3] = 'b';
				strcat(DepthFile, ".bin");

				HANDLE hApproachFind = INVALID_HANDLE_VALUE;
				WIN32_FIND_DATA idx_ffd;
				TCHAR szIdxDir[MAX_PATH] = { 0, };
				char objName[256];
				strcpy(objName, ProcFileName);
				filePathLen = strlen(ProcFileName);
				objName[filePathLen - 4] = '\0';
				sprintf(ApproachPath, "%s\\APPROACH\\%s\\PROCIMG\\*", tBuf, objName);
				MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, ApproachPath, strlen(ApproachPath), szIdxDir, MAX_PATH);
				hApproachFind = FindFirstFile(szIdxDir, &class_ffd);

				while (FindNextFile(hApproachFind, &idx_ffd) != 0){
					//1. Image load
					char ApproachImg[256];
					size_t Applen;
					StringCchLength(idx_ffd.cFileName, MAX_PATH, &Applen);
					WideCharToMultiByte(CP_ACP, 0, idx_ffd.cFileName, 256, ApproachImg, 256, NULL, NULL);

					if (ApproachImg[0] == '.')
						continue;

					//image load
					char appImgFile[256], appDepthFile[256], appAngleFile[256];
					sprintf(appImgFile, "%s\\APPROACH\\%s\\PROCIMG\\%s", tBuf, objName, ApproachImg);

					sprintf(appDepthFile, "%s\\APPROACH\\%s\\PROCDEPTH\\%s", tBuf, objName, ApproachImg);
					int depthwidth, depthheight, depthType;
					filePathLen = strlen(appDepthFile);
					appDepthFile[filePathLen - 1] = 'n';
					appDepthFile[filePathLen - 2] = 'i';
					appDepthFile[filePathLen - 3] = 'b';

					char appAnglePath[256];
					TCHAR szAppAngleDir[MAX_PATH] = { 0, };
					sprintf(appAnglePath, "%s\\APPROACH\\%s\\ANGLE", tBuf, objName);
					MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, appAnglePath, strlen(appAnglePath), szAppAngleDir, MAX_PATH);
					szAppAngleDir[strlen(appAnglePath)] = '\0';
					bool mkdir_check = CreateDirectory(szAppAngleDir, NULL);

					sprintf(appAngleFile, "%s\\%s", appAnglePath, ApproachImg);
					filePathLen = strlen(appAngleFile);
					appAngleFile[filePathLen - 1] = 't';
					appAngleFile[filePathLen - 2] = 'x';
					appAngleFile[filePathLen - 3] = 't';

					FilePath tempPath;
					tempPath.rgbpath = appImgFile;
					tempPath.depthpath = appDepthFile;
					tempPath.angpath = appAngleFile;

					FileList.push_back(tempPath);
				}
			}

		}
	}
}

void Approach_Data_Loader::getData(cv::Mat *rgb, cv::Mat *depth, cv::Mat *rgbOri, int idx){
	rgb->release();
	depth->release();
	rgbOri->release();

	int height_, width_;
	height_ = 250;
	width_ = 400;

	FilePath tempPath = FileList.at(idx);
	//rgb
	cv::Mat img = cv::imread(tempPath.rgbpath);
	cv::Mat tempdataMat(img.rows, img.cols, CV_32FC3);
	if (img.rows == 0)
		return;
	height_ = img.rows;
	width_ = img.cols;
	for (int h = 0; h < img.rows; h++){
		for (int w = 0; w < img.cols; w++){
			for (int c = 0; c < img.channels(); c++){
				tempdataMat.at<float>(c*height_*width_ + width_*h + w) = (float)img.at<cv::Vec3b>(h, w)[c] / 255.0f;
			}
		}
	}

	int depthwidth, depthheight, depthType;
	FILE *fp = fopen(tempPath.depthpath.c_str(), "rb");
	fread(&depthwidth, sizeof(int), 1, fp);
	fread(&depthheight, sizeof(int), 1, fp);
	fread(&depthType, sizeof(int), 1, fp);
	cv::Mat depthMap(depthwidth, depthheight, depthType);
	for (int i = 0; i < depthMap.rows * depthMap.cols; i++)		fread(&depthMap.at<float>(i), sizeof(float), 1, fp);
	fclose(fp);

	*rgb = tempdataMat.clone();
	*depth = depthMap.clone();
	*rgbOri = img.clone();
}

void Approach_Data_Loader::FreeDataAll(){
	FileList.clear();
}

int Approach_Data_Loader::getCount(){
	return FileList.size();
}

void Approach_Data_Loader::writeAngleData(float *ang, int idx){
	std::string path = FileList.at(idx).angpath;
	FILE *fp = fopen(path.c_str(), "w");
	for (int i = 0; i < 9; i++){
		fprintf(fp, "%d\n", (int)ang[i]);
	}
	for (int i = 9; i < 12; i++){
		fprintf(fp, "%f\n", ang[i]);
	}
	fclose(fp);
	printf("%s\n", path.c_str());
}