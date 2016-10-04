#include <vector>

#include "caffe/blob.hpp"
#include "caffe/common.hpp"
#include "caffe/filler.hpp"
#include "caffe/layer.hpp"
#include "caffe/util/math_functions.hpp"
#include "caffe/layers/depth_concat_layer.hpp"

namespace caffe {

	template <typename Dtype>
	void DepthConcatLayer<Dtype>::LayerSetUp(const vector<Blob<Dtype>*>& bottom,
		const vector<Blob<Dtype>*>& top) {
		int batchsize = bottom[0]->shape()[0];
		int first_channel = bottom[0]->shape()[1];
		int first_width = bottom[0]->shape()[2];
		int first_height = bottom[0]->shape()[3];

		int second_channel = bottom[1]->shape()[1];
		int second_width = bottom[1]->shape()[2];
		int second_height = bottom[1]->shape()[3];

		first_dim = first_channel * first_width * first_height;
		second_dim = second_channel * second_width * second_height;

		int total_dim = first_dim + second_dim;
		N_ = total_dim;
	}

	template <typename Dtype>
	void DepthConcatLayer<Dtype>::Reshape(const vector<Blob<Dtype>*>& bottom,
		const vector<Blob<Dtype>*>& top) {
		// Figure out the dimensions
		const int axis = bottom[0]->CanonicalAxisIndex(
			this->layer_param_.spatial_param().axis());
		const int new_K = bottom[0]->count(axis);
		// The first "axis" dimensions are independent inner products; the total
		// number of these is M_, the product over these dimensions.
		M_ = bottom[0]->count(0, axis);
		// The top shape will be the bottom shape with the flattened axes dropped,
		// and replaced by a single axis with dimension num_output (N_).
		vector<int> top_shape = bottom[0]->shape();
		top_shape.resize(axis + 1);
		top_shape[axis] = N_;
		top[0]->Reshape(top_shape);
	}

	template <typename Dtype>
	void DepthConcatLayer<Dtype>::Forward_cpu(const vector<Blob<Dtype>*>& bottom,
		const vector<Blob<Dtype>*>& top) {
		//not implemented
	}

	template <typename Dtype>
	void DepthConcatLayer<Dtype>::Backward_cpu(const vector<Blob<Dtype>*>& top,
		const vector<bool>& propagate_down,
		const vector<Blob<Dtype>*>& bottom) {
		//not implemented
	}

#ifdef CPU_ONLY
	STUB_GPU(SpatialLayer);
#endif

	INSTANTIATE_CLASS(DepthConcatLayer);
	REGISTER_LAYER_CLASS(DepthConcat);

}  // namespace caffe