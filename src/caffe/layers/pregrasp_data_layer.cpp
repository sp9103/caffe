#include <opencv2/core/core.hpp>

#include <vector>

#include "caffe/layers/pregrasp_data_layer.hpp"
#include "caffe/layer.hpp"
#include "caffe/util/io.hpp"

#include <opencv2\opencv.hpp>
#include <conio.h>
#include <strsafe.h>
#include <Windows.h>
#include <time.h>

#define SWAP(a,b,t) (t)=(a), (a)=(b), (b)=(t)

namespace caffe {

	template <typename Dtype>
	void PreGraspDataLayer<Dtype>::DataLayerSetUp(const vector<Blob<Dtype>*>& bottom,
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
		top[1]->Reshape(batch_size_, 1, height_, width_);												//[1] Depth
		std::vector<int> pos_dim(2);
		pos_dim[0] = batch_size_;
		pos_dim[1] = 9;
		top[2]->Reshape(pos_dim);													//[2] Pregrasping postion (label)	

		//전체 로드
		PreGrasp_DataLoadAll(data_path_.c_str());
		CHECK_GT(FileList.size(), 0) << "data is empty";

		//랜덤 박스 생성
		std::random_shuffle(FileList.begin(), FileList.end());
		dataidx = 0;

		stop_thread = false;
		ThreadCount = 4;
		BatchList.clear();
		for (int i = 0; i < ThreadCount; i++){
			LoadThread[i] = std::thread(&PreGraspDataLayer::LoadFuc, this, ThreadCount, i);
		}
	}

	template <typename Dtype>
	void PreGraspDataLayer<Dtype>::Reset(Dtype* data, Dtype* labels, int n) {
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
	void PreGraspDataLayer<Dtype>::set_batch_size(int new_size) {
		/*CHECK(!has_new_data_) <<
		"Can't change batch_size until current data has been consumed.";*/
		batch_size_ = new_size;
	}

	template <typename Dtype>
	void PreGraspDataLayer<Dtype>::Forward_cpu(const vector<Blob<Dtype>*>& bottom,
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
			LoadThread[i] = std::thread(&PreGraspDataLayer::LoadFuc, this, ThreadCount, i);
		}
	}

	template <typename Dtype>
	bool PreGraspDataLayer<Dtype>::comp(const FilePath& s1, const FilePath& s2) {
		return (s1.id > s2.id);
	}

	template <typename Dtype>
	void PreGraspDataLayer<Dtype>::PreGrasp_DataLoadAll(const char* datapath){
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
				char idxsetPath[256];
				sprintf(idxsetPath, "%s\\newIdxSet.txt", tBuf);
				FILE *idxsetfp = fopen(idxsetPath, "r");
				if (idxsetfp == NULL)
					continue;

				FilePath tempPath;
				while (!feof(idxsetfp)){
					int imgIdx, angIdx;
					fscanf(idxsetfp, "%d %d\n", &imgIdx, &angIdx);

					char imgPath[256], angPath[256], depthPath[256];
					sprintf(imgPath, "%s\\PROCESSIMG2\\%d.bmp", tBuf, imgIdx);
					sprintf(angPath, "%s\\ANGLE\\%d.txt", tBuf, angIdx);
					sprintf(depthPath, "%s\\DEPTHMAP2\\%d.bin", tBuf, imgIdx);
					tempPath.image_path = imgPath;
					tempPath.depth_path = depthPath;
					tempPath.ang_path = angPath;

					FileList.push_back(tempPath);
				}
				fclose(idxsetfp);
			}
		}
	}

	template <typename Dtype>
	bool PreGraspDataLayer<Dtype>::fileTypeCheck(char *fileName){
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
	void PreGraspDataLayer<Dtype>::makeRandbox(int *arr, int size){
		for (int i = 0; i < size; i++)
			arr[i] = i;
		for (int i = 0; i < size; i++){
			int tidx = rand() % size;
			int t = arr[i];
			arr[i] = arr[tidx];
			arr[tidx] = t;
		}
	}

	template <typename Dtype>
	void PreGraspDataLayer<Dtype>::LoadFuc(int totalThread, int id){
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
				angMat.at<float>(i) = (float)angBox[i] / angle_max[i] * 180.f;
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
				std::random_shuffle(FileList.begin(), FileList.end());
				dataidx = 0;
				idx_mtx.unlock();
			}

			if (image_blob.size() > 4000)
				break;
		}
	}

	INSTANTIATE_CLASS(PreGraspDataLayer);
	REGISTER_LAYER_CLASS(PreGraspData);

}  // namespace caffe
