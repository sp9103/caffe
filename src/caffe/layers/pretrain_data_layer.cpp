#include <opencv2/core/core.hpp>

#include <vector>

#include "caffe/layers/pretrain_data_layer.hpp"
#include "caffe/layer.hpp"
#include "caffe/util/io.hpp"

#include <opencv2\opencv.hpp>
#include <conio.h>
#include <strsafe.h>
#include <Windows.h>
#include <time.h>

#define _USE_MATH_DEFINES
#include <math.h>

#define SWAP(a,b,t) (t)=(a), (a)=(b), (b)=(t)

namespace caffe {

	template <typename Dtype>
	void PretrainDataLayer<Dtype>::DataLayerSetUp(const vector<Blob<Dtype>*>& bottom,
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

		output_dim_ = 2;

		top[0]->Reshape(batch_size_, channels_, height_, width_);					//[0] RGB

		std::vector<int> pos_dim(2);
		pos_dim[0] = batch_size_;
		pos_dim[1] = output_dim_;															//angle : 9
		top[1]->Reshape(pos_dim);													//[2] Pregrasping postion (label)

		//전체 로드
		Pretrain_DataLoadAll(data_path_.c_str());
		CHECK_GT(FileList.size(), 0) << "data is empty";

		//랜덤 박스 생성
		std::random_shuffle(FileList.begin(), FileList.end());
		LoadThread = std::thread(&PretrainDataLayer::LoadFuc, this);
	}

	template <typename Dtype>
	void PretrainDataLayer<Dtype>::Reset(Dtype* data, Dtype* labels, int n) {
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
	void PretrainDataLayer<Dtype>::set_batch_size(int new_size) {
		/*CHECK(!has_new_data_) <<
		"Can't change batch_size until current data has been consumed.";*/
		batch_size_ = new_size;
	}

	template <typename Dtype>
	void PretrainDataLayer<Dtype>::Forward_cpu(const vector<Blob<Dtype>*>& bottom,
		const vector<Blob<Dtype>*>& top) {
		Dtype* rgb_data = top[0]->mutable_cpu_data();					//[0] RGB
		Dtype* pos_data = top[1]->mutable_cpu_data();					//[1] Depth

		for (int i = 0; i < batch_size_; i++){
			save_mtx.lock();
			cv::Mat posMat;
			cv::Mat	rgbImg;
			if (pos_blob.size() > 1){
				posMat = *pos_blob.begin();
				rgbImg = *image_blob.begin();

				pos_blob.pop_front();
				image_blob.pop_front();
			}
			else{
				i--;
				save_mtx.unlock();
				continue;
			}
			save_mtx.unlock();

			caffe_copy(channels_ * height_ * width_, rgbImg.ptr<Dtype>(0), rgb_data);
			caffe_copy(output_dim_, posMat.ptr<Dtype>(0), pos_data);

			rgb_data += top[0]->offset(1);
			pos_data += top[1]->offset(1);
		}
	}

	template <typename Dtype>
	void PretrainDataLayer<Dtype>::Pretrain_DataLoadAll(const char* datapath){
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

					char rgbImgFile[256], posPath[256];
					FILE *fp;
					int filePathLen;
					//image load
					strcpy(rgbImgFile, procDir);
					filePathLen = strlen(rgbImgFile);
					rgbImgFile[filePathLen - 1] = '\0';
					strcat(rgbImgFile, ProcFileName);

					//pos load
					sprintf(posPath, "%s\\ObjPos\\%s", tBuf, ProcFileName);
					filePathLen = strlen(posPath);
					posPath[filePathLen - 1] = 't';
					posPath[filePathLen - 2] = 'x';
					posPath[filePathLen - 3] = 't';

					FilePath tempPath;
					tempPath.image_path = rgbImgFile;
					tempPath.pos_path = posPath;

					FileList.push_back(tempPath);
					if (data_limit_ < FileList.size() && data_limit_ > 0)
						return;
				}
			}
		}
	}

	template <typename Dtype>
	bool PretrainDataLayer<Dtype>::fileTypeCheck(char *fileName){
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
	void PretrainDataLayer<Dtype>::LoadFuc(){
		const int ThreadLimit = 1000;
		std::thread FileLoadThread[ThreadLimit];
		int ThreadIdx = 0, dataidx = 0;

		for (int i = 0; i < ThreadLimit; i++){
			FilePath srcPath = FileList.at(dataidx++);
			FileLoadThread[i] = std::thread(&PretrainDataLayer::ReadFuc, this, srcPath);
		}

		while (1){
			int label_count = pos_blob.size();
			if (label_count < ThreadLimit){
				FilePath srcPath = FileList.at(dataidx++);
				//불러오기 쓰레드
				if (FileLoadThread[ThreadIdx].joinable())
					FileLoadThread[ThreadIdx].join();
				else
					printf("noting.\n");
				FileLoadThread[ThreadIdx] = std::thread(&PretrainDataLayer::ReadFuc, this, srcPath);
				ThreadIdx = (ThreadIdx + 1) % ThreadLimit;

				//초과됬을때
				if (dataidx >= FileList.size()){
					dataidx = 0;
					std::random_shuffle(FileList.begin(), FileList.end());
				}
			}
		}
	}

	template <typename Dtype>
	void PretrainDataLayer<Dtype>::ReadFuc(FilePath src){
		//RGB load
		std::string imageFilaPath = src.image_path;
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
		std::string posFilaPath = src.pos_path;
		FILE *fp = fopen(posFilaPath.c_str(), "r");
		if (fp == NULL)
			return;
		cv::Mat posMat(output_dim_, 1, CV_32FC1);
		float labelBox[2];
		int angBox[2];
		fscanf(fp, "%f %f\n", &posMat.at<float>(0), &posMat.at<float>(1));
		fclose(fp);

		save_mtx.lock();
		image_blob.push_back(tempdataMat);
		pos_blob.push_back(posMat);
		save_mtx.unlock();
	}

	INSTANTIATE_CLASS(PretrainDataLayer);
	REGISTER_LAYER_CLASS(PretrainData);

}  // namespace caffe
