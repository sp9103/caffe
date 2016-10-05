#include <opencv2/core/core.hpp>

#include <vector>

#include "caffe/layers/ik_data_layer.hpp"
#include "caffe/layer.hpp"
#include "caffe/util/io.hpp"

#include <opencv2\opencv.hpp>
#include <conio.h>
#include <strsafe.h>
#include <Windows.h>
#include <time.h>

#define _USE_MATH_DEFINES
#include <math.h>

namespace caffe {

template <typename Dtype>
void IKDataLayer<Dtype>::DataLayerSetUp(const vector<Blob<Dtype>*>& bottom,
     const vector<Blob<Dtype>*>& top) {
	batch_size_ = this->layer_param_.sp_unsupervised_data_param().batch_size();
	channels_ = this->layer_param_.sp_unsupervised_data_param().channels();
	height_ = this->layer_param_.sp_unsupervised_data_param().height();
	width_ = this->layer_param_.sp_unsupervised_data_param().width();

	data_path_ = this->layer_param_.sp_unsupervised_data_param().data_path();
	data_limit_ = this->layer_param_.sp_unsupervised_data_param().data_limit();

  size_ = channels_ * height_ * width_;
  CHECK_GT(batch_size_ * size_, 0) <<
      "batch_size, channels, height, and width must be specified and"
      " positive in memory_data_param";

  top[0]->Reshape(batch_size_, channels_, height_, width_);					//[0] RGB
  top[1]->Reshape(batch_size_, 1, height_, width_);                                                //[1] Depth

  std::vector<int> ang_dim(2);
  ang_dim[0] = batch_size_;
  ang_dim[1] = 9;
  top[2]->Reshape(ang_dim);													//[2] Angle (label)

  //��ü �ε�
  IK_DataLoadAll(data_path_.c_str());
  CHECK_GT(FileList.size(), 0) << "data is empty";

  //���� �ڽ� ����
  dataidx = 0;
  std::random_shuffle(FileList.begin(), FileList.end());

  stop_thread = false;
  ThreadCount = 4;
  BatchList.clear();
  for (int i = 0; i < ThreadCount; i++){
	  LoadThread[i] = std::thread(&IKDataLayer::LoadFuc, this, ThreadCount, i);
  }
}

template <typename Dtype>
void IKDataLayer<Dtype>::Reset(Dtype* data, Dtype* labels, int n) {
  CHECK(data);
  CHECK(labels);
  CHECK_EQ(n % batch_size_, 0) << "n must be a multiple of batch size";
  // Warn with transformation parameters since a memory array is meant to
  // be generic and no transformations are done with Reset().
  if (this->layer_param_.has_transform_param()) {
    LOG(WARNING) << this->type() << " does not transform array data on Reset()";
  }
  n_ = n;
}

template <typename Dtype>
void IKDataLayer<Dtype>::set_batch_size(int new_size) {
  /*CHECK(!has_new_data_) <<
      "Can't change batch_size until current data has been consumed.";*/
  batch_size_ = new_size;
}

template <typename Dtype>
void IKDataLayer<Dtype>::Forward_cpu(const vector<Blob<Dtype>*>& bottom,
	const vector<Blob<Dtype>*>& top) {
	Dtype* rgb_data = top[0]->mutable_cpu_data();					//[0] RGB
	Dtype* depth_data = top[1]->mutable_cpu_data();					//[1] Depth
	Dtype* ang_data = top[2]->mutable_cpu_data();					//[2] ang postion (label)

	stop_thread = true;
	for (int i = 0; i < ThreadCount; i++){
		LoadThread[i].join();
	}

	for (int i = 0; i < batch_size_; i++){
		cv::Mat angMat = *ang_blob.begin();
		cv::Mat	rgbImg = *image_blob.begin();
		cv::Mat depth = *depth_blob.begin();

		caffe_copy(channels_ * height_ * width_, rgbImg.ptr<Dtype>(0), rgb_data);
		caffe_copy(height_ * width_, depth.ptr<Dtype>(0), depth_data);
		caffe_copy(9, angMat.ptr<Dtype>(0), ang_data);

		rgb_data += top[0]->offset(1);
		depth_data += top[1]->offset(1);
		ang_data += top[2]->offset(1);

		ang_blob.pop_front();
		image_blob.pop_front();
		depth_blob.pop_front();
	}

	stop_thread = false;
	BatchList.clear();
	for (int i = 0; i < ThreadCount; i++){
		LoadThread[i] = std::thread(&IKDataLayer::LoadFuc, this, ThreadCount, i);
	}
}

template <typename Dtype>
void IKDataLayer<Dtype>::IK_DataLoadAll(const char* datapath){
	WIN32_FIND_DATA ffd;
	HANDLE hFind = INVALID_HANDLE_VALUE;
	TCHAR szDir[MAX_PATH] = { 0, };

	MultiByteToWideChar(CP_ACP, MB_PRECOMPOSED, datapath, strlen(datapath), szDir, MAX_PATH);
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
				//1. process Image load
				FilePath tempPath;
				char ProcFileName[256];
				size_t Proclen;
				StringCchLength(class_ffd.cFileName, MAX_PATH, &Proclen);
				WideCharToMultiByte(CP_ACP, 0, class_ffd.cFileName, 256, ProcFileName, 256, NULL, NULL);

				if (ProcFileName[0] == '.')
					continue;

				char AngDataFile[256], ProcImageFile[256], DepthFile[256], EndFile[256];
				int imgCount = image_blob.size();
				int filePathLen;
				//ProcImage �о����
				strcpy(ProcImageFile, procDir);
				filePathLen = strlen(ProcImageFile);
				ProcImageFile[filePathLen - 1] = '\0';
				strcat(ProcImageFile, ProcFileName);
				//image_path.push_back(ProcImageFile);
				tempPath.image_path = ProcImageFile;

				//2. angle �о����
				sprintf(AngDataFile, "%s\\ANGLE\\%s", tBuf, ProcFileName);
				filePathLen = strlen(AngDataFile);
				AngDataFile[filePathLen - 1] = 't';
				AngDataFile[filePathLen - 2] = 'x';
				AngDataFile[filePathLen - 3] = 't';
				//ang_path.push_back(AngDataFile);
				tempPath.ang_path = AngDataFile;

				//3.depth �о����
				sprintf(DepthFile, "%s\\DEPTHMAP2\\%s", tBuf, ProcFileName);
				filePathLen = strlen(DepthFile);
				DepthFile[filePathLen - 1] = 'n';
				DepthFile[filePathLen - 2] = 'i';
				DepthFile[filePathLen - 3] = 'b';
				//depth_path.push_back(DepthFile);
				tempPath.depth_path = DepthFile;

				char idBuf[256];
				strcpy(idBuf, ProcFileName);
				filePathLen = strlen(idBuf);
				idBuf[filePathLen - 4] = '\0';
				tempPath.id = atoi(idBuf);

				FileList.push_back(tempPath);
			}

		}
	}
}

template <typename Dtype>
bool IKDataLayer<Dtype>::fileTypeCheck(char *fileName){
	size_t fileLen;
	fileLen = strlen(fileName);

	if (fileLen < 5)
		return false;

	if (fileName[fileLen - 1] != 'g' && fileName[fileLen - 1] != 'p')
		return false;
	if (fileName[fileLen - 2] != 'p' && fileName[fileLen - 2] != 'm')
		return false;
	if (fileName[fileLen - 3] != 'j' && fileName[fileLen - 3] != 'b')
		return false;

	return true;
}

template <typename Dtype>
void IKDataLayer<Dtype>::LoadFuc(int totalThread, int id){
	//angle min max
	int angle_max[9] = { 251000, 251000, 251000, 251000, 151875, 151875, 4095, 4095, 4095 };

	while (!stop_thread || ang_blob.size() < batch_size_){
		FILE *fp;
		int depthwidth, depthheight, depthType;

		idx_mtx.lock();
		int myIdx = dataidx;
		dataidx = (dataidx + 1) % FileList.size();
		FilePath tempPath = FileList.at(myIdx);
		idx_mtx.unlock();

		//RGB load
		std::string imageFilaPath = /*image_path.at(myIdx)*/tempPath.image_path;
		cv::Mat img = cv::imread(imageFilaPath);
		cv::Mat tempdataMat(height_, width_, CV_32FC3);
		for (int h = 0; h < img.rows; h++){
			for (int w = 0; w < img.cols; w++){
				for (int c = 0; c < img.channels(); c++){
					tempdataMat.at<float>(c*height_*width_ + width_*h + w) = (float)img.at<cv::Vec3b>(h, w)[c] / 255.0f;
				}
			}
		}

		//Angle load
		std::string angleFilaPath = /*ang_path.at(myIdx)*/tempPath.ang_path;
		fp = fopen(angleFilaPath.c_str(), "r");
		if (fp == NULL)
			continue;
		cv::Mat angMat(9, 1, CV_32FC1);
		cv::Mat labelMat(9, 1, CV_32FC1);
		int angBox[9];
		bool angError = false;
		for (int i = 0; i < 9; i++){
			fscanf(fp, "%d", &angBox[i]);
			angMat.at<float>(i) = (float)angBox[i] / angle_max[i] *  180.f;
			labelMat.at<float>(i) = angMat.at<float>(i);
			if (angBox[i] >= 250950 || angBox[i] <= -250950){
				angError = true;
				break;
			}
		}
		if (angError){
			/*ang_path.erase(ang_path.begin() + myIdx);
			depth_path.erase(depth_path.begin() + myIdx);
			image_path.erase(image_path.begin() + myIdx);*/
			FileList.erase(FileList.begin() + myIdx);
			continue;
		}
		fclose(fp);

		//Depth load
		std::string depthFilePath = /*depth_path.at(myIdx)*/tempPath.depth_path;
		fp = fopen(depthFilePath.c_str(), "rb");
		if (fp == NULL)
			continue;
		fread(&depthwidth, sizeof(int), 1, fp);
		fread(&depthheight, sizeof(int), 1, fp);
		fread(&depthType, sizeof(int), 1, fp);
		cv::Mat depthMap(depthheight, depthwidth, depthType);
		for (int i = 0; i < depthMap.rows * depthMap.cols; i++)        fread(&depthMap.at<float>(i), sizeof(float), 1, fp);
		fclose(fp);

		//store
		save_mtx.lock();
		image_blob.push_back(tempdataMat);
		depth_blob.push_back(depthMap);
		ang_blob.push_back(angMat);
		labelMat.push_back(labelMat);
		if (BatchList.size() < batch_size_)
			BatchList.push_back(tempPath);
		save_mtx.unlock();

		if (dataidx >= this->FileList.size()){
			idx_mtx.lock();
			dataidx = 0;
			std::random_shuffle(FileList.begin(), FileList.end());
			idx_mtx.unlock();
		}

		if (image_blob.size() > 4000)
			break;
	}
}

INSTANTIATE_CLASS(IKDataLayer);
REGISTER_LAYER_CLASS(IKData);

}  // namespace caffe
