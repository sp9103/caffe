#ifndef CAFFE_APPROACH_DATA_LAYER_HPP_
#define CAFFE_APPROACH_DATA_LAYER_HPP_

#include <vector>

#include "caffe/blob.hpp"
#include "caffe/layer.hpp"
#include "caffe/proto/caffe.pb.h"

namespace caffe {

	/**
	* @brief Provides data to the Net from memory.
	*
	* TODO(dox): thorough documentation for Forward and proto params.
	*/
	//image, depth, pregrasping pos, COM 
	template <typename Dtype>
	class ApproachDataLayer : public BaseDataLayer<Dtype> {
	public:
		explicit ApproachDataLayer(const LayerParameter& param)
			: BaseDataLayer<Dtype>(param) {}
		virtual void DataLayerSetUp(const vector<Blob<Dtype>*>& bottom,
			const vector<Blob<Dtype>*>& top);

		virtual inline const char* type() const { return "ApproachData"; }
		virtual inline int ExactNumBottomBlobs() const { return 0; }

		// Reset should accept const pointers, but can't, because the memory
		//  will be given to Blob, which is mutable
		void Reset(Dtype* data, Dtype* label, int n);
		void set_batch_size(int new_size);

		int batch_size() { return batch_size_; }
		int channels() { return channels_; }
		int height() { return height_; }
		int width() { return width_; }
		int data_limit() { return data_limit_; }

	protected:
		virtual void Forward_cpu(const vector<Blob<Dtype>*>& bottom,
			const vector<Blob<Dtype>*>& top);

		typedef struct path_{
			std::string image_path;
			std::string depth_path;
			std::string ang_path;
		}FilePath;

		void Approach_DataLoadAll(const char* datapath);
		bool fileTypeCheck(char *fileName);
		void makeRandbox(int *arr, int size);
		void LoadFuc(int totalThread, int id);

		int batch_size_, channels_, height_, width_, size_;
		int n_;
		int data_limit_;

		std::string data_path_;

		std::list<cv::Mat> image_blob;						//rgb image
		std::list<cv::Mat> depth_blob;						//distance
		std::list<cv::Mat> ang_blob;			//pregrasping pos (image idx, pos)

		std::vector<FilePath> FileList;

		std::mutex idx_mtx, save_mtx;
		std::thread LoadThread[4];
		int ThreadCount;
		bool stop_thread;

		int *randbox;
		int dataidx;
	};

}  // namespace caffe

#endif  // CAFFE_APPROACH_DATA_LAYER_HPP_
