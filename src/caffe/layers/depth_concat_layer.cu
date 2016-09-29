#include <vector>

#include "caffe/blob.hpp"
#include "caffe/common.hpp"
#include "caffe/filler.hpp"
#include "caffe/layer.hpp"
#include "caffe/util/math_functions.hpp"
#include "caffe/layers/depth_concat_layer.hpp"

#include <opencv2\opencv.hpp>

namespace caffe {

	template <typename Dtype>
	__global__ void kernel_features_concat(const int num, const int total_dim, const int dim_1,
		const int dim_2, const Dtype* data_1, const Dtype* data_2,
		Dtype* out) {
		CUDA_KERNEL_LOOP(index, num) {
			int batch_idx = index / total_dim;
			int inner_idx = index % total_dim;

			if (inner_idx < dim_1){
				out[index] = data_1[batch_idx * dim_1 + inner_idx];
			}
			else{
				out[index] = data_2[batch_idx * dim_2 + (inner_idx - dim_1)];
			}
		}
	}

	template <typename Dtype>
	__global__ void kernel_features_seperate(const int num, const int total_dim, const int dim_1,
		const int dim_2, const Dtype* data, Dtype* out_1, Dtype* out_2) {
		CUDA_KERNEL_LOOP(index, num) {
			int batch_idx = index / total_dim;
			int inner_idx = index % total_dim;

			if (inner_idx < dim_1){
				out_1[batch_idx * dim_1 + inner_idx] = data[index];
			}
			else{
				out_2[batch_idx * dim_2 + (inner_idx - dim_1)] = data[index];
			}
		}
	}

	template <typename Dtype>
	void DepthConcatLayer<Dtype>::Forward_gpu(const vector<Blob<Dtype>*>& bottom,
		const vector<Blob<Dtype>*>& top) {
		Dtype* top_data = top[0]->mutable_gpu_data();
		const Dtype* first_bot_data = bottom[0]->gpu_data();
		const Dtype* second_bot_data = bottom[1]->gpu_data();
		int count = top[0]->count();

		kernel_features_concat<Dtype> << <CAFFE_GET_BLOCKS(count),
			CAFFE_CUDA_NUM_THREADS >> >(count, first_dim + second_dim, first_dim, second_dim,
			first_bot_data, second_bot_data, top_data);
	}

	template <typename Dtype>
	void DepthConcatLayer<Dtype>::Backward_gpu(const vector<Blob<Dtype>*>& top,
		const vector<bool>& propagate_down,
		const vector<Blob<Dtype>*>& bottom) {
		const Dtype* top_diff = top[0]->gpu_diff();
		Dtype* first_bot_diff = bottom[0]->mutable_gpu_diff();
		Dtype* second_bot_diff = bottom[1]->mutable_gpu_diff();
		int count = top[0]->count();

		kernel_features_seperate<Dtype> << <CAFFE_GET_BLOCKS(count),
			CAFFE_CUDA_NUM_THREADS >> >(count, first_dim + second_dim, first_dim, second_dim,
			top_diff, first_bot_diff, second_bot_diff);
	}

	INSTANTIATE_LAYER_GPU_FUNCS(DepthConcatLayer);

}  // namespace caffe
