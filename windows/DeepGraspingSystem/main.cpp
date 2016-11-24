#include <stdio.h>
#include <caffe\caffe.hpp>

#include "HandTracker\MOGbasedTracker.h"
#include "Robot\RobotManager.h"
#include "Kinect\KinectMangerThread.h"

#define APPROACH_NET_PATH "..\\caffe\\APP_AlexMDN\\deploy_approach.prototxt"
//Approach net weight file path
#define APPROACH_NET_TRAINRESULT "..\\caffe\\APP_AlexMDN\\snapshot\\APP_AlexMDN_iter_33030.caffemodel"
#define PREGRASP_NET_PATH "..\\caffe\\Pregrasp_AlexNet\\deploy_pregrasp.prototxt"
//Pregrasp net weight file path
#define PREGRASP_NET_TRAINRESULT "..\\caffe\\Pregrasp_AlexNet\\snapshot_1118\\Pregrasp_AlexNet_iter_23368.caffemodel"
//#define PREGRASP_NET_TRAINRESULT "..\\caffe\\Pregrasp_AlexNet\\snapshot_1115\\Pregrasp_AlexNet_iter_41084.caffemodel"

using namespace caffe;

void resultToRobotMotion(const std::vector<caffe::Blob<float>*>& src, int *dst);
void MDNresultToRobotMotion(const std::vector<caffe::Blob<float>*>& src, int *dst);
void rgbConvertCaffeType(cv::Mat src, cv::Mat *dst);
void depthVis(cv::Mat src, char* windowName);
float calcMaxDiff(int *src1, int*src2);

int main(){
	//0-1. Kinect Initialize
	int width = WIDTH;
	int height = HEIGHT;
	cv::Rect				RobotROI((KINECT_DEPTH_WIDTH - width) / 2, (KINECT_DEPTH_HEIGHT - height) / 2 + 20, width, height);
	KinectMangerThread		kinect;
	kinect.Initialize(RobotROI);

	//0-2. Robot Initialze
	getch();
	RobotManager			robot;
	robot.Initialize(4, 3);

	//robot Initial move
	robot.TorqueOn();
	robot.safeRelease();
	printf("if system ready, press any key to console\n");
	getch();
	cv::Mat RgbBack = kinect.getImg();
	cv::Mat DepthBack = kinect.getDepth();
	cv::cvtColor(RgbBack, RgbBack, CV_BGRA2BGR);
	MOGbasedTracker tracker;
	tracker.InsertBackGround(RgbBack, DepthBack);

	//0-3. Deepnet initialize
	// mode setting - CPU/GPU
	Caffe::set_mode(Caffe::GPU);
	// gpu device number
	int device_id = 0;
	Caffe::SetDevice(device_id);

	//1.Approaching network load 
	Net<float> approach_net(APPROACH_NET_PATH, caffe::TEST);
	approach_net.CopyTrainedLayersFrom(APPROACH_NET_TRAINRESULT);

	//2.Pregrasping network load
	Net<float> pregrasp_net(PREGRASP_NET_PATH, caffe::TEST);
	pregrasp_net.CopyTrainedLayersFrom(PREGRASP_NET_TRAINRESULT);

	//RUN
	Blob<float> rgbBlob(1, 3, HEIGHT, WIDTH);
	Blob<float> depthBlob(1, 1, HEIGHT, WIDTH);
	int robotMotion[9];
	float loss;
	printf("Start calculate network output, press 's' key to opencv window\n");
	while (1){
		cv::Mat kinectRGB = kinect.getImg();
		cv::Mat kinectDEPTH = kinect.getDepth();

		depthVis(kinectDEPTH, "depth");
		cv::imshow("ROI", kinectRGB);
		char key = cv::waitKey(10);

		if (key == 's'){

			////Approaching network
			////Mat -> Blob
			cv::cvtColor(kinectRGB, kinectRGB, CV_BGRA2BGR);
			cv::Mat caffeRgb;
			rgbConvertCaffeType(kinectRGB, &caffeRgb);
			memcpy(rgbBlob.mutable_cpu_data(), caffeRgb.ptr<float>(0), sizeof(float) * HEIGHT * WIDTH * CHANNEL);
			memcpy(depthBlob.mutable_cpu_data(), kinectDEPTH.ptr<float>(0), sizeof(float) * HEIGHT * WIDTH);
			vector<Blob<float>*> input_vec;				//입력 RGB, DEPTH
			input_vec.push_back(&rgbBlob);
			input_vec.push_back(&depthBlob);

			////Approaching
			const vector<Blob<float>*>& result_approach = approach_net.Forward(input_vec, &loss);
			MDNresultToRobotMotion(result_approach, robotMotion);
			robot.Approaching(robotMotion);
			printf("if motion end, press any key\n");


			//Pregrasp
			while (1){
				input_vec.clear();
				cv::Mat kinectRGBPregrasp = kinect.getImg();
				cv::Mat kinectDEPTHPregrasp = kinect.getDepth();
				cv::Mat procImg, procDepth;
				tracker.calcImage(kinectRGBPregrasp, kinectDEPTHPregrasp, &procImg, &procDepth);

				if (procImg.rows == 0) continue;

				depthVis(procDepth, "procDepth");
				cv::imshow("procImg", procImg);
				char pregraspKey = cv::waitKey(10);
				rgbConvertCaffeType(procImg, &caffeRgb);
				memcpy(rgbBlob.mutable_cpu_data(), caffeRgb.ptr<float>(0), sizeof(float) * HEIGHT * WIDTH * CHANNEL);
				memcpy(depthBlob.mutable_cpu_data(), procDepth.ptr<float>(0), sizeof(float) * HEIGHT * WIDTH);
				input_vec.push_back(&rgbBlob);
				input_vec.push_back(&depthBlob);
				const vector<Blob<float>*>& result_pregrasp = pregrasp_net.Forward(input_vec, &loss);
				resultToRobotMotion(result_pregrasp, robotMotion);

				int presntState[NUM_XEL];
				robot.getPresState(presntState);
				float maxdiff = calcMaxDiff(robotMotion, presntState);
				printf("max diff %f\n", maxdiff);
				if (robotMotion[1] < 0 && maxdiff < 2.f){
					//실제 잡기.
					robot.grasp();
					break;
				}
				else if (robotMotion[1] > 0 && maxdiff < 2.f){
					//실제 잡기.
					robot.grasp();
					break;
				}
				else if (pregraspKey == 27){
					//실제 잡기.
					robot.grasp();
					break;
				}

				printf("Move next step robot motion press any key\n");
				robot.Move(robotMotion);

			}

			//잡은 이후
			/*printf("after grasping, press any key\n");
			getch();*/
			robot.Lift();
			printf("after Lift, press any key\n");
			getch();
			robot.safeRelease();
		}
		else if (key == 'q')
			break;
	}
	robot.TorqueOff();

	robot.DeInitialize();
	kinect.Deinitialize();

	return 0;
}

void resultToRobotMotion(const std::vector<caffe::Blob<float>*>& src, int *dst){
	int angle_max[9] = { 251000, 251000, 251000, 251000, 151875, 151875, 4095, 4095, 4095 };
	float outputData[9];
	memcpy(outputData, src.at(0)->cpu_data(), sizeof(float) * 9);
	for (int j = 0; j < DATADIM; j++){
		dst[j] = (int)(outputData[j] / 180.f * angle_max[j]);
	}
}

void MDNresultToRobotMotion(const std::vector<caffe::Blob<float>*>& src, int *dst){
	const int class_size = 5;
	int angle_max[9] = { 251000, 251000, 251000, 251000, 151875, 151875, 4095, 4095, 4095 };
	float outputData[55];
	memcpy(outputData, src.at(0)->cpu_data(), sizeof(float) * 55);
	float alphaMax = -1.f;
	for (int c = 0; c < 5; c++){
		float alpha = outputData[11*c];
		float tempOutput[11];
		memcpy(tempOutput, &outputData[11 * c], sizeof(float) * 11);

		if (alphaMax < alpha){
			alphaMax = alpha;
			float inv = 1.f;
			for (int j = 0; j < DATADIM; j++){
				dst[j] = (int)(tempOutput[j+1] / 180.f * angle_max[j] * 100.f) * inv;

				if (j == 1 && tempOutput[j + 1] < 0)
					inv *= -1.f;
			}
		}
	}
}

void rgbConvertCaffeType(cv::Mat src, cv::Mat *dst){
	dst->create(src.rows, src.cols, CV_32FC3);
	for (int h = 0; h < src.rows; h++){
		for (int w = 0; w < src.cols; w++){
			for (int c = 0; c < src.channels(); c++){
				dst->at<float>(c*src.rows*src.cols + src.cols*h + w) = (float)src.at<cv::Vec3b>(h, w)[c] / 255.0f;
			}
		}
	}
}

void depthVis(cv::Mat src, char* windowName){
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
	cv::imshow(windowName, temp);
}

float calcMaxDiff(int *src1, int*src2){
	const int angle_max[9] = { 251000, 251000, 251000, 251000, 151875, 151875, 4095, 4095, 4095 };
	float max = -1;

	for (int i = 0; i < NUM_XEL; i++){
		float diff = (float)abs(src1[i] - src2[i]) / (float)angle_max[i] * 180;

		if (max < diff)
			max = diff;
	}

	return max;
}