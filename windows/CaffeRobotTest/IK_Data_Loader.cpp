#include "IK_Data_Loader.h"


IK_Data_Loader::IK_Data_Loader()
{
	height_ = width_ = 160;
}


IK_Data_Loader::~IK_Data_Loader()
{
}

int IK_Data_Loader::getCount(){
	return FileList.size();
}

void IK_Data_Loader::LoadDataAll(char *RootPath){
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

			//////////////////////////////////////////////////////////
			//새경로 생성
			TCHAR distDir[MAX_PATH] = { 0, };
			char new_path[256];
			sprintf(new_path, "%s\\ANGLE_INFERENCE", tBuf);
			MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, new_path, strlen(new_path), distDir, MAX_PATH);
			bool mkdir_check = CreateDirectory(distDir, NULL);
			//////////////////////////////////////////////////////////

			while (FindNextFile(hDataFind, &class_ffd) != 0){
				//1. process Image load
				char ProcFileName[256];
				size_t Proclen;
				StringCchLength(class_ffd.cFileName, MAX_PATH, &Proclen);
				WideCharToMultiByte(CP_ACP, 0, class_ffd.cFileName, 256, ProcFileName, 256, NULL, NULL);

				if (ProcFileName[0] == '.')
					continue;

				char AngDataFile[256], ProcImageFile[256], DepthFile[256], RgbOriFile[256];
				FilePath tempPath;
				FILE *fp;
				int filePathLen;
				//ProcImage 읽어오기
				strcpy(ProcImageFile, procDir);
				filePathLen = strlen(ProcImageFile);
				ProcImageFile[filePathLen - 1] = '\0';
				strcat(ProcImageFile, ProcFileName);
				tempPath.rgbpath = ProcImageFile;

				//RGB Ori
				sprintf(RgbOriFile, "%s\\RGB\\%s", tBuf, ProcFileName);
				tempPath.rgbOriPath = RgbOriFile;

				//2. angle 읽어오기
				sprintf(AngDataFile, "%s\\ANGLE\\%s", tBuf, ProcFileName);
				filePathLen = strlen(AngDataFile);
				AngDataFile[filePathLen - 1] = 't';
				AngDataFile[filePathLen - 2] = 'x';
				AngDataFile[filePathLen - 3] = 't';
				tempPath.angpath = AngDataFile;

				//3.Depth 읽어오기
				sprintf(DepthFile, "%s\\DEPTHMAP2\\%s", tBuf, ProcFileName);
				filePathLen = strlen(DepthFile);
				DepthFile[filePathLen - 1] = 'n';
				DepthFile[filePathLen - 2] = 'i';
				DepthFile[filePathLen - 3] = 'b';
				tempPath.depthpath = DepthFile;

				////////////////////////////////////////////////////////////
				//새로 저장할 파일 경로 생성
				sprintf(AngDataFile, "%s\\ANGLE_INFERENCE\\%s", tBuf, ProcFileName);
				filePathLen = strlen(AngDataFile);
				AngDataFile[filePathLen - 1] = 't';
				AngDataFile[filePathLen - 2] = 'x';
				AngDataFile[filePathLen - 3] = 't';
				tempPath.newPath = AngDataFile;
				////////////////////////////////////////////////////////////

				//4.저장
				FileList.push_back(tempPath);
			}

		}
	}
}

void IK_Data_Loader::getData(cv::Mat *rgb, cv::Mat *depth, cv::Mat *ang, cv::Mat *rgbOri, float *dist, cv::Point3f *endeff, int idx){
	rgb->release();
	depth->release();
	ang->release();
	rgbOri->release();

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
	*rgbOri = img.clone();
}

void IK_Data_Loader::FreeDataAll(){
	FileList.clear();
}

void IK_Data_Loader::createEndeffector(char *RootPath, armsdk::Kinematics *kin){
	WIN32_FIND_DATA ffd;
	HANDLE hFind = INVALID_HANDLE_VALUE;
	TCHAR szDir[MAX_PATH] = { 0, };
	//angle min max
	int angle_max[9] = { 251000, 251000, 251000, 251000, 151875, 151875, 4095, 4095, 4095 };
	const float div_factor = 100.f;
	vecd angd;
	veci angi(6);

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
			strcat(procDir, "\\PROCESSIMG\\*");
			MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, procDir, strlen(procDir), szProcDir, MAX_PATH);
			hDataFind = FindFirstFile(szProcDir, &class_ffd);

			while (FindNextFile(hDataFind, &class_ffd) != 0){
				//1. process Image load
				char ProcFileName[256];
				size_t Proclen;
				StringCchLength(class_ffd.cFileName, MAX_PATH, &Proclen);
				WideCharToMultiByte(CP_ACP, 0, class_ffd.cFileName, 256, ProcFileName, 256, NULL, NULL);

				if (ProcFileName[0] == '.')
					continue;

				char AngDataFile[256];
				FILE *fp;
				int filePathLen;

				//2. angle 읽어오기
				sprintf(AngDataFile, "%s\\ANGLE\\%s", tBuf, ProcFileName);
				filePathLen = strlen(AngDataFile);
				AngDataFile[filePathLen - 1] = 't';
				AngDataFile[filePathLen - 2] = 'x';
				AngDataFile[filePathLen - 3] = 't';
				fp = fopen(AngDataFile, "r");
				if (fp == NULL)		continue;
				cv::Mat angMat(9, 1, CV_32FC1);
				cv::Mat angOriMat(9, 1, CV_32FC1);
				int angBox[9];
				bool angError = false;
				for (int i = 0; i < 9; i++){
					fscanf(fp, "%d", &angBox[i]);
					angMat.at<float>(i) = (float)angBox[i] / angle_max[i] * M_PI;
					angOriMat.at<float>(i) = (float)angBox[i];
					if (angBox[i] >= 250950 || angBox[i] <= -250950){
						angError = true;
						break;
					}
				}
				if (angError)		continue;
				fclose(fp);

				//endeffector
				armsdk::Pose3D endeffectorOri;
				TCHAR distDir[MAX_PATH] = { 0, };
				char endeffFile[256];
				sprintf(endeffFile, "%s\\ENDEFFECTOR", tBuf);
				MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, endeffFile, strlen(endeffFile), distDir, MAX_PATH);
				bool mkdir_check = CreateDirectory(distDir, NULL);
				for (int j = 0; j < 6; j++)			angi[j] = angBox[j];
				angd = kin->Value2Rad(angi);
				kin->Forward(angd, &endeffectorOri);
				sprintf(endeffFile, "%s\\ENDEFFECTOR\\%s", tBuf, ProcFileName);
				filePathLen = strlen(endeffFile);
				endeffFile[filePathLen - 1] = 't';
				endeffFile[filePathLen - 2] = 'x';
				endeffFile[filePathLen - 3] = 't';
				fp = fopen(endeffFile, "w");
				fprintf(fp, "%f\n", endeffectorOri.x);
				fprintf(fp, "%f\n", endeffectorOri.y);
				fprintf(fp, "%f\n", endeffectorOri.z);
				fclose(fp);
			}

		}
	}
}

void IK_Data_Loader::writeAngle(int *src, int idx){
	FILE *fp = fopen(FileList.at(idx).newPath.c_str(), "w");
	for (int i = 0; i < 9; i++)
		fprintf(fp, "%d\n", src[i]);
	fclose(fp);
}