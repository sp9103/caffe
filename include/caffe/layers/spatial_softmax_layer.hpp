#ifndef CAFFE_SPATIAL_SOFTMAX_LAYER_HPP_
#define CAFFE_SPATIAL_SOFTMAX_LAYER_HPP_

#include <vector>

#include <string>
#include <utility>
#include <vector>

#include "caffe/blob.hpp"
#include "caffe/common.hpp"
#include "caffe/layer.hpp"
#include "caffe/proto/caffe.pb.h"

namespace caffe {

	template <typename Dtype>
	class SpatialSoftmaxLayer : public Layer<Dtype> {
	public:
		explicit SpatialSoftmaxLayer(const LayerParameter& param)
			: Layer<Dtype>(param) {}
		virtual void LayerSetUp(const vector<Blob<Dtype>*>& bottom,
			const vector<Blob<Dtype>*>& top);
		virtual void Reshape(const vector<Blob<Dtype>*>& bottom,
			const vector<Blob<Dtype>*>& top);

		virtual inline const char* type() const { return "SpatialSoftmax"; }
		//virtual inline int ExactNumBottomBlobs() const { return 1; }
		virtual inline int ExactNumTopBlobs() const { return 1; }

	protected:
		virtual void Forward_cpu(const vector<Blob<Dtype>*>& bottom,
			const vector<Blob<Dtype>*>& top);
		virtual void Forward_gpu(const vector<Blob<Dtype>*>& bottom,
			const vector<Blob<Dtype>*>& top);
		virtual void Backward_cpu(const vector<Blob<Dtype>*>& top,
			const vector<bool>& propagate_down, const vector<Blob<Dtype>*>& bottom);
		virtual void Backward_gpu(const vector<Blob<Dtype>*>& top,
			const vector<bool>& propagate_down, const vector<Blob<Dtype>*>& bottom);

		int M_;
		int K_;
		int N_;
		float alpha_;

		Blob<Dtype> backwardTemp_;
		Blob<Dtype> softmaxResult_;
		Blob<Dtype>	maxValue_;						//softmax의 결과물과 max_value의 결과물

		bool is_visualize;
	};

}  // namespace caffe

#endif  // CAFFE_SPATIAL_SOFTMAX_LAYER_HPP_
