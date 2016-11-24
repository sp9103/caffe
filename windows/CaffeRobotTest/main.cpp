#include <stdio.h>

#include <caffe\caffe.hpp>

#include "ARMSDK\include\ARMSDK.h"
#include "IK_Data_Loader.h"
//#include "ListLoader.h"
//#include "Approach_Data_Loader.h"
#include "Pregrasp_Data_Loader.h"
//#include "SimulatorControl.h"

#define HEIGHT			250
#define WIDTH			400
#define CHANNEL			3
#define ClASSNUM		5
#define DATADIM			9

//#define SOLVER "deploy_IK_Net_fc.prototxt"
//#define TRAINRESULT "weight_fc\\IK_Net_iter_50000.caffemodel"
#define SOLVER "deploy_val.prototxt"
#define TRAINRESULT "..\\caffe\\IK_AlexNet\\snapshot_1104\\IK_AlexNet_iter_8404.caffemodel"
#define RIGHT_ARM_USE

using namespace caffe;

typedef struct robotResult_{
	int label[DATADIM];
	int ans[DATADIM];
	float Error;
	cv::Mat rgb;
	cv::Mat depth;
}robotResult;

typedef struct netResult_{
	float presX;
	float presY;
	float presZ;
	float Dist;
	float angle[9];
	float loss;
}NetResult;

cv::Mat depthLoad(char *fileName){
	int depthwidth, depthheight, depthType;
	FILE *fp = fopen(fileName, "rb");
	fread(&depthwidth, sizeof(int), 1, fp);
	fread(&depthheight, sizeof(int), 1, fp);
	fread(&depthType, sizeof(int), 1, fp);
	cv::Mat depthMap(depthheight, depthwidth, depthType);
	for (int i = 0; i < depthMap.rows * depthMap.cols; i++)		fread(&depthMap.at<float>(i), sizeof(float), 1, fp);
	fclose(fp);

	return depthMap.clone();
}

bool ErrorMaxSort(robotResult first, robotResult second);
void depthVis(cv::Mat src);

//int main(){
//	//ListLoader TestDataLoader;
//	//TestDataLoader.LoadDataAll("C:\\TempCaffeDebugData\\list.txt");
//	int angle_max[9] = { 251000, 251000, 251000, 251000, 151875, 151875, 4095, 4095, 4095 };
//	std::vector<robotResult> result_vec;
//
//	armsdk::RobotInfo robot;
//	armsdk::Kinematics kin;
//#ifdef RIGHT_ARM_USE
//	//RightArm
//	robot.AddJoint(0.0, ML_PI_2, 0.0, 0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 1);
//	robot.AddJoint(0.0, -ML_PI_2, 0.0, 0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 3);
//	robot.AddJoint(30.0, -ML_PI_2, 246.0, 0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 5);
//	robot.AddJoint(-30.0, ML_PI_2, 0.0, ML_PI_2, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 7);
//	robot.AddJoint(0.0, -ML_PI_2, 216.0, 0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 9);
//	robot.AddJoint(0.0, ML_PI_2, 0.0, 0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 11);
//#elif defined LEFT_ARM_USE
//	//Leftarm
//	robot.AddJoint(0.0, -ML_PI_2, 0.0, 0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 2);
//	robot.AddJoint(0.0, ML_PI_2, 0.0, 0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 4);
//	robot.AddJoint(30.0, ML_PI_2, 246.0, 0.0, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 6);
//	robot.AddJoint(-30.0, -ML_PI_2, 0.0, -ML_PI_2, ML_PI, -ML_PI, 251000, -251000, ML_PI, -ML_PI, 8);
//	robot.AddJoint(0.0, ML_PI_2, 216.0, 0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 10);
//	robot.AddJoint(0.0, -ML_PI_2, 0.0, 0.0, ML_PI, -ML_PI, 151875, -151875, ML_PI, -ML_PI, 12);
//#endif
//	kin.InitRobot(&robot);
//
//	// mode setting - CPU/GPU
//	Caffe::set_mode(Caffe::GPU);
//
//	// gpu device number
//	int device_id = 0;
//	Caffe::SetDevice(device_id);
//
//	//network test setting
//	Net<float> caffe_test_net(SOLVER, caffe::TEST);
//
//	// caffemodel(weight)
//	caffe_test_net.CopyTrainedLayersFrom(TRAINRESULT);
//
//	IK_Data_Loader TestDataLoader;
//	TestDataLoader.LoadDataAll(/*"D:\\self_supervisedData\\TEST"*/"G:\\IK_Data\\SINGLE");
//
//	//Test & 통계 내기;
//	Blob<float> rgbBlob(1, 3, HEIGHT, WIDTH);
//	Blob<float> depthBlob(1, 1, HEIGHT, WIDTH);
//	vector<int> distDim(2);
//	//distDim[0] = distDim[1] = 1;
//	Blob<float> distBlob(1,1,1,1);
//	armsdk::Pose3D endeffectorOri, endeffectorNet;
//	vecd angd;
//	veci angi(6);
//	float TotalmaxDist = 0.f, TotalminDist = FLT_MAX;
//	int angBox[DATADIM];
//	cv::Point3f averEff = cv::Point3f(0, 0, 0);
//	cv::Point3f MaxEff;
//	float averEffDistance = 0;
//	float MaxEffDistance = 0;
//	float TotalmaxErrorDist = 0.f;
//	std::vector<NetResult> NetResultVec;
//
//	for (int i = 0; i < TestDataLoader.getCount(); i++){
//		cv::Mat rgbimg, depthImg, angVec, rgbori;
//		TestDataLoader.getData(&rgbimg, &depthImg, &angVec, &rgbori, NULL, NULL, i);
//		if (rgbimg.rows == 0)
//			continue;
//		
//		//forward kinematices
//		for (int j = 0; j < DATADIM; j++)	angBox[j] = (int)angVec.at<float>(j);
//		for (int j = 0; j < 6; j++)			angi[j] = (int)angVec.at<float>(j);
//
//		angd = kin.Value2Rad(angi);
//		kin.Forward(angd, &endeffectorOri);
//
//		//RGB Mat -> RGB Blob
//		//Depth Mat -> Depth Blob
//		memcpy(rgbBlob.mutable_cpu_data(), rgbimg.ptr<float>(0), sizeof(float) * HEIGHT * WIDTH * CHANNEL);
//		memcpy(depthBlob.mutable_cpu_data(), depthImg.ptr<float>(0), sizeof(float) * HEIGHT * WIDTH);
//		//입력 형식 맞춰주기
//		vector<Blob<float>*> input_vec;				//입력 RGB, DEPTH
//		input_vec.push_back(&rgbBlob);
//		input_vec.push_back(&depthBlob);
//		//네트워크 출력 계산
//		float loss, alphaMax = 0.0f;
//		float outputAlphaMax[DATADIM];
//		const vector<Blob<float>*>& result = caffe_test_net.Forward(input_vec, &loss);
//		//float minDist = FLT_MAX;
//		//float MINAngDist = 99999.f;
//		//float outputMinDist[9];
//		//for (int c = 0; c < ClASSNUM; c++){
//		//	float outputData[11];
//		//	memcpy(outputData, &result.at(0)->cpu_data()[11*c], sizeof(float) * 11);
//		//	float alpha = outputData[0];
//		//	float sigma = outputData[10];
//
//		//	float traData[DATADIM];
//		//	for (int j = 0; j < DATADIM; j++){
//		//		traData[j] = outputData[j + 1] * angle_max[j] / M_PI;
//		//	}
//
//		//	if (alphaMax < alpha){
//		//		alphaMax = alpha;
//		//		for (int j = 0; j < DATADIM; j++)
//		//			outputAlphaMax[j] = traData[j];
//		//	}
//
//		//	float presAngDist = 0;
//		//	for (int j = 0; j < 9; j++){
//		//		presAngDist += abs((float)angBox[j] - traData[j]);
//		//	}
//		//	if (MINAngDist > presAngDist){
//		//		for (int j = 0; j < 9; j++)
//		//			outputMinDist[j] = traData[j];
//		//		MINAngDist = presAngDist;
//		//	}
//		//}
//		//////////////////////////////////////////
//		float outputData[9];
//		memcpy(outputData, result.at(0)->cpu_data(), sizeof(float) * 9);
//		for (int j = 0; j < DATADIM; j++)
//			outputAlphaMax[j] = outputData[j] / 180.f * angle_max[j];
//		/////////////////////////////////////////
//
//		for (int j = 0; j < 6; j++)	angi[j] = (int)outputAlphaMax[j];/*angi[j] = (int)outputMinDist[j]*/;
//		angd = kin.Value2Rad(angi);
//		kin.Forward(angd, &endeffectorNet);
//		
//		NetResult tempNetResult;
//		tempNetResult.presX = abs(endeffectorOri.x - endeffectorNet.x);
//		tempNetResult.presY = abs(endeffectorOri.y - endeffectorNet.y);
//		tempNetResult.presZ = abs(endeffectorOri.z - endeffectorNet.z);
//		tempNetResult.Dist = sqrt(pow(tempNetResult.presX, 2) + pow(tempNetResult.presY, 2) + pow(tempNetResult.presZ, 2));
//		for (int a = 0; a < 9; a++){
//			float labeAngle = (float)angBox[a] / (float)angle_max[a] * 180.f;
//			tempNetResult.angle[a] = abs(outputData[a] - labeAngle);
//		}
//		tempNetResult.loss = loss;
//		NetResultVec.push_back(tempNetResult);
//		printf("distance : %fmm\n", tempNetResult.Dist);
//		if (TotalmaxErrorDist < tempNetResult.Dist)	TotalmaxErrorDist = tempNetResult.Dist;
//
//		robotResult tempResult;
//		for (int j = 0; j < DATADIM; j++){
//			tempResult.ans[j] = (int)outputAlphaMax[j];
//			//tempResult.ans[j] = (int)outputMinDist[j];
//			tempResult.label[j] = angBox[j];
//		}
//		tempResult.Error = tempNetResult.Dist;
//		tempResult.rgb = rgbori.clone();
//		tempResult.depth = depthImg.clone();
//		result_vec.push_back(tempResult);
//
//		//TestDataLoader.writeAngle(tempResult.ans, i);
//
//		if (TotalmaxDist < tempNetResult.Dist)	TotalmaxDist = tempNetResult.Dist;
//		if (TotalminDist > tempNetResult.Dist)	TotalminDist = tempNetResult.Dist;
//	}
//
//	float averDist = 0.0f;
//	float averX, averY, averZ, averloss;
//	float averAngle[9], averTotalAngle;
//	averX = averY = averZ = averTotalAngle = averloss = 0.0f;
//	memset(averAngle, 0, sizeof(float) * 9);
//	for (int i = 0; i < NetResultVec.size(); i++){
//		averX += NetResultVec.at(i).presX / NetResultVec.size();
//		averY += NetResultVec.at(i).presY / NetResultVec.size();
//		averZ += NetResultVec.at(i).presZ / NetResultVec.size();
//		averDist += NetResultVec.at(i).Dist / NetResultVec.size();
//		averloss += NetResultVec.at(i).loss / NetResultVec.size();
//		for (int j = 0; j < 9; j++){
//			averAngle[j] += NetResultVec.at(i).angle[j] / NetResultVec.size();
//		}
//	}
//	printf("Dist : %fmm\n", averDist);
//	printf("DistX : %fmm\n", averX);
//	printf("DistY : %fmm\n", averY);
//	printf("DistZ : %fmm\n", averZ);
//	printf("Average Loss : %f\n", averloss);
//	printf("Angle : ");
//	for (int i = 0; i < 9; i++){
//		averTotalAngle += averAngle[i] / 9;
//		printf("%.3f ", averAngle[i]);
//	}
//	printf("total Angle %f", averTotalAngle);
//	printf("\nMAXDist : %fmm\n", TotalmaxErrorDist);
//
//	TestDataLoader.FreeDataAll();
//
//	////////////////////////////////////////////////////////////////////////////////
//	//simulator 구동
//	/*SimulatorControl simul;
//	simul.Initialize();*/
//
//	////data calculator
//	//std::vector<cv::Mat>	rgbList;
//	//Blob<float> rgbBlob(1, 3, HEIGHT, WIDTH);
//	//Blob<float> depthBlob(1, 1, HEIGHT, WIDTH);
//	//char targetObj[256] = "D:\\ApproachData\\TRAIN\\pokari";
//
//	////1. 바닥 뎁스 로드
//	//char depthbackpath[256];
//	//sprintf(depthbackpath, "%s\\background.bin", targetObj);
//	//cv::Mat backDepth = depthLoad(depthbackpath);
//	//sprintf(depthbackpath, "%s\\background.bmp", targetObj);
//	//cv::Mat backRGB = cv::imread(depthbackpath);
//
//	//char objDepthpath[256];
//	//TCHAR szDir[MAX_PATH] = { 0, };
//	//WIN32_FIND_DATA ffd;
//	//sprintf(objDepthpath, "%s\\DEPTH\\*", targetObj);
//	//MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, objDepthpath, strlen(objDepthpath), szDir, MAX_PATH);
//	//HANDLE hFind = FindFirstFile(szDir, &ffd);
//	//while (FindNextFile(hFind, &ffd) != 0){
//
//	//	//Tchar to char
//	//	char ccFileName[256];
//	//	size_t len;
//	//	WideCharToMultiByte(CP_ACP, 0, ffd.cFileName, 256, ccFileName, 256, NULL, NULL);
//	//	if (ccFileName[0] == '.')	continue;
//	//	printf("directory : %s load.\n", ccFileName);
//
//	//	char bmppath[256];
//	//	sprintf(bmppath, "%s\\RGB\\%s", targetObj, ccFileName);
//	//	int bmplen = strlen(bmppath);
//	//	bmppath[bmplen - 7] = 'b';
//	//	bmppath[bmplen - 6] = 'm';
//	//	bmppath[bmplen - 5] = 'p';
//	//	bmppath[bmplen - 4] = '\0';
//	//	cv::Mat objRGB = cv::imread(bmppath);
//	//	if (objRGB.rows == 0)
//	//		continue;
//
//	//	if (ccFileName[0] != '.'){
//
//	//		if (strcmp(ccFileName, "12.bin.bin") != 0){
//	//			continue;
//	//		}
//
//	//		//object depthpus
//	//		char depthMatpath[256];
//	//		strcpy(depthMatpath, objDepthpath);
//	//		int length = strlen(depthMatpath);
//	//		depthMatpath[length - 1] = '\0';
//	//		strcat(depthMatpath, ccFileName);
//	//		cv::Mat objDepthMat = depthLoad(depthMatpath);
//
//	//		//Directory make
//	//		char MotionDir[256];
//	//		char procDir[256];
//	//		char motionDepthDir[256];
//	//		sprintf(MotionDir, "%s\\APPROACH\\%s", targetObj, ccFileName);
//	//		length = strlen(MotionDir);
//	//		MotionDir[length - 8] = '\0';
//	//		strcat(MotionDir, "\\");
//	//		strcpy(procDir, MotionDir);
//	//		strcpy(motionDepthDir, MotionDir);
//	//		strcat(MotionDir, "MOTION");
//	//		TCHAR szMotionDir[MAX_PATH] = { 0, };
//	//		MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, MotionDir, strlen(MotionDir), szMotionDir, MAX_PATH);
//	//		CreateDirectory(szMotionDir, NULL);
//
//	//		WIN32_FIND_DATA ffd_proc;
//	//		TCHAR szProcDir[MAX_PATH] = { 0, };
//	//		strcat(procDir, "PROCIMG\\*");
//	//		MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, procDir, strlen(procDir), szProcDir, MAX_PATH);
//	//		HANDLE hProcFind = FindFirstFile(szProcDir, &ffd_proc);
//	//		while (FindNextFile(hProcFind, &ffd_proc) != 0){
//	//			char ccProcFileName[256];
//	//			size_t lenProc;
//	//			WideCharToMultiByte(CP_ACP, 0, ffd_proc.cFileName, 256, ccProcFileName, 256, NULL, NULL);
//	//			if (ccProcFileName[0] == '.')	continue;
//	//			//프로세싱 이미지 로드
//	//			char procImgPath[256];
//	//			strcpy(procImgPath, procDir);
//	//			int lenProcPath = strlen(procImgPath);
//	//			procImgPath[lenProcPath - 1] = '\0';
//	//			strcat(procImgPath, ccProcFileName);
//
//	//			cv::Mat inputProc = cv::imread(procImgPath);
//	//			cv::Mat tempdataMat(inputProc.rows, inputProc.cols, CV_32FC3);
//	//			for (int h = 0; h < inputProc.rows; h++){
//	//				for (int w = 0; w < inputProc.cols; w++){
//	//					for (int c = 0; c < inputProc.channels(); c++){
//	//						tempdataMat.at<float>(c*inputProc.rows*inputProc.cols + inputProc.cols*h + w) = (float)inputProc.at<cv::Vec3b>(h, w)[c] / 255.0f;
//	//					}
//	//				}
//	//			}
//	//			rgbList.push_back(inputProc.clone());
//	//			result_img.push_back(inputProc.clone());
//
//	//			//프로세싱 뎁스로드
//	//			char tempMotionDir[256];
//	//			strcpy(tempMotionDir, motionDepthDir);
//	//			strcat(tempMotionDir, "DEPTH\\");
//	//			strcat(tempMotionDir, ccProcFileName);
//	//			lenProcPath = strlen(tempMotionDir);
//	//			tempMotionDir[lenProcPath - 3] = 'b';
//	//			tempMotionDir[lenProcPath - 2] = 'i';
//	//			tempMotionDir[lenProcPath - 1] = 'n';
//	//			cv::Mat inputDepth = depthLoad(tempMotionDir);
//
//	//			//뎁스 정리
//	//			int countfix = 0;
//	//			cv::Mat tempVis = inputProc.clone();
//	//			for (int h = 0; h < inputDepth.rows; h++){
//	//				for (int w = 0; w < inputDepth.cols; w++){
//	//					cv::Vec3b procVal = inputProc.at<cv::Vec3b>(h, w);
//	//					cv::Vec3b backVal = backRGB.at<cv::Vec3b>(h, w);
//	//					cv::Vec3b difVal = cv::Vec3b(abs(procVal[0] - backVal[0]), abs(procVal[1] - backVal[1]), abs(procVal[2] - backVal[2]));
//	//					if (difVal[0] < 5 && difVal[1] < 5 && difVal[2] < 5){
//	//						countfix++;
//	//						inputDepth.at<float>(h, w) = backDepth.at<float>(h, w);
//	//						tempVis.at<cv::Vec3b>(h, w) = cv::Vec3b(0, 0, 0);
//	//					}
//	//				}
//	//			}
//
//	//			///////////////////////////////////////
//	//			float alphaSet[ClASSNUM], sigmaSet[ClASSNUM];
//	//			float outputSet[ClASSNUM][DATADIM];
//	//			///////////////////////////////////////
//
//	//			//딥넷으로 계산
//	//			memcpy(rgbBlob.mutable_cpu_data(), tempdataMat.ptr<float>(0), sizeof(float) * HEIGHT * WIDTH * CHANNEL);
//	//			memcpy(depthBlob.mutable_cpu_data(), inputDepth.ptr<float>(0), sizeof(float) * HEIGHT * WIDTH);
//	//			vector<Blob<float>*> input_vec;				//입력 RGB, DEPTH
//	//			input_vec.push_back(&rgbBlob);
//	//			input_vec.push_back(&depthBlob);
//	//			float loss, alphaMax = 0.0f;
//	//			float outputAlphaMax[DATADIM];
//	//			const vector<Blob<float>*>& result = caffe_test_net.Forward(input_vec, &loss);
//	//			for (int c = 0; c < ClASSNUM; c++){
//	//				float outputData[11];
//	//				memcpy(outputData, &result.at(0)->cpu_data()[11 * c], sizeof(float) * 11);
//	//				float alpha = outputData[0];
//	//				float sigma = outputData[10];
//
//	//				alphaSet[c] = alpha;
//	//				sigmaSet[c] = sigma;
//	//				for (int j = 0; j < DATADIM; j++)
//	//					outputSet[c][j] = outputData[j + 1] * angle_max[j] / M_PI;
//
//	//				if (alphaMax < alpha){
//	//					alphaMax = alpha;
//	//					for (int j = 0; j < DATADIM; j++)
//	//						outputAlphaMax[j] = outputData[j + 1] * angle_max[j] / M_PI;
//	//				}
//	//			}
//
//	//			/*memcpy(outputAlphaMax, result.at(0)->cpu_data(), sizeof(float) * DATADIM);
//	//			for (int i = 0; i < DATADIM; i++){
//	//				outputAlphaMax[i] = outputAlphaMax[i] * angle_max[i] / M_PI;
//	//			}*/
//
//	//			//결과 쓰기
//	//			char motionfpFile[256];
//	//			sprintf(motionfpFile, "%s\\%\s", MotionDir, ccProcFileName);
//	//			lenProcPath = strlen(motionfpFile);
//	//			motionfpFile[lenProcPath - 3] = 't';
//	//			motionfpFile[lenProcPath - 2] = 'x';
//	//			motionfpFile[lenProcPath - 1] = 't';
//	//			FILE *motionFP = fopen(motionfpFile, "w");
//	//			robotResult temp;
//	//			for (int i = 0; i < DATADIM; i++){
//	//				fprintf(motionFP, "%d\n", (int)outputAlphaMax[i]);
//	//				temp.ans[i] = (int)outputAlphaMax[i];
//	//				temp.label[i] = (int)outputAlphaMax[i];
//	//			}
//	//			//임시 시뮬레이터 렌더링
//	//			simul.renderData(temp.ans);
//	//			cv::imshow("img", inputProc);
//	//			cv::waitKey(0);
//
//	//			//if (alphaMax < 0.7){
//	//			//	for (int c = 0; c < ClASSNUM; c++){
//	//			//		for (int i = 0; i < DATADIM; i++){
//	//			//			temp.ans[i] = (int)outputSet[c][i];
//	//			//		}
//	//			//		simul.renderData(temp.ans);
//	//			//		cv::imshow("img", inputProc);
//	//			//		cv::waitKey(0);
//	//			//	}
//	//			//}
//
//	//			result_vec.push_back(temp);
//	//			fclose(motionFP);
//	//		}
//	//	}
//	//}
//	//////////////////////////////////////////////////////////////////////////////////////
//
//	std::sort(result_vec.begin(), result_vec.end(), ErrorMaxSort);
//	for (int i = 0; i < result_vec.size(); i++){
//		int idx = rand() & result_vec.size();
//		int labelAngle[DATADIM], resultAngle[DATADIM];
//		memcpy(labelAngle, result_vec.at(i).label, sizeof(int) * DATADIM);
//		memcpy(resultAngle, result_vec.at(i).ans, sizeof(int) * DATADIM);
//
//		//simul.renderData(labelAngle);
//		cv::imshow("img", result_vec.at(i).rgb);
//		depthVis(result_vec.at(i).depth);
//		//cv::waitKey(0);
//		//simul.renderData(resultAngle);
//		cv::waitKey(0);
//	}
//
//	//simul.Deinitialize();
//
//	return 0;
//}
//
//approach net labe 계산
int main(){
	int angle_max[9] = { 251000, 251000, 251000, 251000, 151875, 151875, 4095, 4095, 4095 };

	// mode setting - CPU/GPU
	Caffe::set_mode(Caffe::GPU);

	// gpu device number
	int device_id = 0;
	Caffe::SetDevice(device_id);

	//network test setting
	Net<float> caffe_test_net(SOLVER, caffe::TEST);

	// caffemodel(weight)
	caffe_test_net.CopyTrainedLayersFrom(TRAINRESULT);

	//Test & 통계 내기;
	Blob<float> rgbBlob(1, 3, HEIGHT, WIDTH);
	Blob<float> depthBlob(1, 1, HEIGHT, WIDTH);

	//data load
	//Approach_Data_Loader dataLoader;
	//dataLoader.LoadDataAll("E:\\Approach_data");
	Pregrasp_Data_Loader dataLoader;
	dataLoader.LoadDataAll("M:\\Pregrasp_data_test\\TRAIN");

		//simulator 구동
	/*SimulatorControl simul;
	simul.Initialize();*/

	for (int i = 0; i < dataLoader.getCount(); i++){

		cv::Mat rgbimg, depthImg, angVec, rgbori;
		dataLoader.getData(&rgbimg, &depthImg, &rgbori, i);
		if (rgbimg.rows == 0 || depthImg.rows == 0)
			continue;

		//RGB Mat -> RGB Blob
		//Depth Mat -> Depth Blob
		memcpy(rgbBlob.mutable_cpu_data(), rgbimg.ptr<float>(0), sizeof(float) * HEIGHT * WIDTH * CHANNEL);
		memcpy(depthBlob.mutable_cpu_data(), depthImg.ptr<float>(0), sizeof(float) * HEIGHT * WIDTH);
		//입력 형식 맞춰주기
		vector<Blob<float>*> input_vec;				//입력 RGB, DEPTH
		input_vec.push_back(&rgbBlob);
		input_vec.push_back(&depthBlob);
		//네트워크 출력 계산
		float loss, alphaMax = 0.0f;
		const vector<Blob<float>*>& result = caffe_test_net.Forward(input_vec, &loss);

		float outputData[9], outputAlphaMax[9];
		int rendData[9];
		//if (abs(outputData[1]) )
		memcpy(outputData, result.at(0)->cpu_data(), sizeof(float) * 9);
		for (int j = 0; j < DATADIM; j++){
			outputAlphaMax[j] = outputData[j] / 180.f * angle_max[j];
			rendData[j] = (int)outputAlphaMax[j];
		}

		dataLoader.writeAngleData(outputAlphaMax, i);
		//cv::imshow("rgb", rgbori);
		//depthVis(depthImg);
		//simul.renderData(rendData);
		//cv::waitKey(0);
	}
}

bool ErrorMaxSort(robotResult first, robotResult second){
	return first.Error > second.Error;
}

void depthVis(cv::Mat src){
	cv::Mat temp = src.clone();

	float max = -1.f, min = 9999.f;
	for (int h = 0; h < temp.rows; h++){
		for (int w = 0; w < temp.cols; w++){
			float val = temp.at<float>(h, w);
			if (val > max)	max = val;
			if (val < min)	min = val;
		}
	}
	for (int h = 0; h < temp.rows; h++){
		for (int w = 0; w < temp.cols; w++){
			float val = temp.at<float>(h, w);
			temp.at<float>(h, w) = (val - min) / (max - min);
		}
	}
	cv::imshow("depth", temp);
}