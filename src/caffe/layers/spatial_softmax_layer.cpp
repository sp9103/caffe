#include <vector>

#include "caffe/blob.hpp"
#include "caffe/common.hpp"
#include "caffe/filler.hpp"
#include "caffe/layer.hpp"
#include "caffe/util/math_functions.hpp"
#include "caffe/layers/spatial_softmax_layer.hpp"

namespace caffe {

	template <typename Dtype>
	void SpatialSoftmaxLayer<Dtype>::LayerSetUp(const vector<Blob<Dtype>*>& bottom,
		const vector<Blob<Dtype>*>& top) {
		//const int num_output = this->layer_param_.spatial_param().num_output();
		const int num_output = bottom[0]->shape()[1] * 2;
		N_ = num_output;
		const int axis = bottom[0]->CanonicalAxisIndex(
			this->layer_param_.spatial_param().axis());
		is_visualize = this->layer_param_.spatial_param().visualize();

		K_ = bottom[0]->count(axis);

		alpha_ = layer_param_.spatial_param().alpha();
	}

	template <typename Dtype>
	void SpatialSoftmaxLayer<Dtype>::Reshape(const vector<Blob<Dtype>*>& bottom,
		const vector<Blob<Dtype>*>& top) {
		// Figure out the dimensions
		const int axis = bottom[0]->CanonicalAxisIndex(
			this->layer_param_.spatial_param().axis());
		const int new_K = bottom[0]->count(axis);
		CHECK_EQ(K_, new_K)
			<< "Input size incompatible with inner product parameters.";
		// The first "axis" dimensions are independent inner products; the total
		// number of these is M_, the product over these dimensions.
		M_ = bottom[0]->count(0, axis);
		// The top shape will be the bottom shape with the flattened axes dropped,
		// and replaced by a single axis with dimension num_output (N_).
		vector<int> top_shape = bottom[0]->shape();
		top_shape.resize(axis + 1);
		top_shape[axis] = N_;
		top[0]->Reshape(top_shape);

		//softmax result blob allocation
		vector<int> scale_dims = bottom[0]->shape();
		softmaxResult_.Reshape(scale_dims);

		//max result blob allocation
		vector<int> max_dims = bottom[0]->shape();
		max_dims[2] = max_dims[3] = 1;
		maxValue_.Reshape(max_dims);
		backwardTemp_.Reshape(max_dims);
	}

	template <typename Dtype>
	void SpatialSoftmaxLayer<Dtype>::Forward_cpu(const vector<Blob<Dtype>*>& bottom,
		const vector<Blob<Dtype>*>& top) {
		const Dtype* bottom_data = bottom[0]->cpu_data();
		Dtype* top_data = top[0]->mutable_cpu_data();
		const int top_count = top[0]->count();

		///////////////////////////parallel softmax;
		Dtype* max_data = maxValue_.mutable_cpu_data();
		Dtype* softmax_result = softmaxResult_.mutable_cpu_data();

		const int bottom_size = bottom[0]->shape()[2] * bottom[0]->shape()[3];			//109*109
		const int feature_count = bottom[0]->count() / bottom_size;						//�� ���� 32*60

		for (int i = 0; i < feature_count; i++){
			//extract max
			max_data[i] = softmax_result[0];
			for (int m = 0; m < bottom_size; m++)
				max_data[i] = std::max(max_data[i],
				softmax_result[m]);			//���ĸ� �ƽø��� ������.

			//subtract max
			for (int m = 0; m < bottom_size; m++)
				softmax_result[m] -= max_data[i];

			//exponential
			caffe_exp<Dtype>(bottom_size, softmax_result, softmax_result);

			//div exponential
			max_data[i] = 0;
			for (int m = 0; m < bottom_size; m++)
				max_data[i] += softmax_result[m];
			for (int m = 0; m < bottom_size; m++)
				softmax_result[m] /= max_data[i];

			//bottom_data += bottom_size;	//���� ���ĸ����� �̵�.
			softmax_result += bottom_size;	//���� ���ĸ����� �̵�.
		}

		///////////////////////////extract maxvalue position
		const Dtype* softmax_data = softmaxResult_.cpu_data();

		const int bottom_num = softmaxResult_.num();					//batch size
		const int map_count = softmaxResult_.shape()[1];				//feature map ���� -> 32��
		const int map_size = softmaxResult_.shape()[2];				//�� �ʿ� ������ -> 109*109 ���簢���̴ϱ� �ϳ��� ����
		for (int n = 0; n < bottom_num; ++n) {
			for (int k = 0; k < map_count; k++){
				float tx, ty;
				float sum_check = 0.0f;
				tx = ty = 0.0f;

				for (int h = 0; h < map_size; h++){
					for (int w = 0; w < map_size; w++){
						const int index = (k*map_size*map_size) + h * map_size + w;
						sum_check += softmax_data[index];
						tx += softmax_data[index] * w;
						ty += softmax_data[index] * h;
					}
				}

				//x��ǥ & y��ǥ ���
				top_data[k * 2 + 0] = tx;
				top_data[k * 2 + 1] = ty;
			}

			// compute offset										//��ġ����� 1 �̻��϶� �������� �ٲ�
			softmax_data += softmaxResult_.offset(1);
			top_data += top[0]->offset(1);
		}

	}

	template <typename Dtype>
	void SpatialSoftmaxLayer<Dtype>::Backward_cpu(const vector<Blob<Dtype>*>& top,
		const vector<bool>& propagate_down,
		const vector<Blob<Dtype>*>& bottom) {
		if (!propagate_down[0]) {			//pooling layer���� ����
			return;
		}

		//extract position backpropagation
		const Dtype* top_diff = top[0]->cpu_diff();

		Dtype* bottom_diff = bottom[0]->mutable_cpu_diff();
		const int bottom_height = bottom[0]->shape()[2];
		const int bottom_width = bottom[0]->shape()[3];
		const int bottom_size = bottom_height * bottom_width;			//109*109
		caffe_set(bottom[0]->count(), (Dtype)0, bottom_diff);			//�ϴ� 0���� �ʱ�ȭ
		for (int i = 0; i < top[0]->count() / 2; i++){					//bottom diff�� ������ŭ ����
			for (int h = 0; h < bottom_height; h++){
				for (int w = 0; w < bottom_width; w++){
					const int idx = h * bottom_width + w;
					bottom_diff[idx] = w * top_diff[2 * i + 0] + h * top_diff[2 * i + 1];
				}
			}

			bottom_diff += bottom_size;
		}

		//softmax backpropagation
		bottom_diff = bottom[0]->mutable_cpu_diff();
		//caffe_copy(109 * 109, bottom_diff, tempMap);

		const int batchSize = bottom[0]->shape()[0];
		const int nChannels = bottom[0]->shape()[1];
		const Dtype* softmax_data = softmaxResult_.cpu_data();
		Dtype* max_data = maxValue_.mutable_cpu_data();
		for (int i = 0; i < batchSize; ++i) {											//�� ��ġ�� ����������
			Dtype* bottom_slice = bottom_diff + i * bottom_size * nChannels;
			caffe_set(nChannels, (Dtype)0, max_data);

			// compute dot(top_diff, top_data) and subtract them from the bottom diff
			for (int k = 0; k < nChannels; ++k) {						//inner_num : 109*109;
				for (int j = 0; j < bottom_size; j++)					//compute dot product
					max_data[k] += bottom_slice[k*bottom_size + j] * softmax_data[k*bottom_size + j];

				for (int j = 0; j < bottom_size; j++)					//subtract them
					bottom_slice[k*bottom_size + j] -= max_data[k];
			}
		}
		// elementwise multiplication
		caffe_mul(top[0]->count(), bottom_diff, softmax_data, bottom_diff);		//top data�� bottom_diff�� elementwise�� ����
	}

#ifdef CPU_ONLY
	STUB_GPU(SpatialSoftmaxLayer);
#endif

	INSTANTIATE_CLASS(SpatialSoftmaxLayer);
	REGISTER_LAYER_CLASS(SpatialSoftmax);

}  // namespace caffe
