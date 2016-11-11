#include <vector>

#include "caffe/layer.hpp"
#include "caffe/util/io.hpp"
#include "caffe/util/math_functions.hpp"
#include "caffe/layers/mdn_loss_layer.hpp"

#include <opencv2\opencv.hpp>

namespace caffe {

template <typename Dtype>
void MDNLossLayer<Dtype>::Reshape(
  const vector<Blob<Dtype>*>& bottom, const vector<Blob<Dtype>*>& top) {
	//MDN���� bottom�� label�� ũ�Ⱑ �� ���� �ʿ� ����
  /*CHECK_EQ(bottom[0]->num(), bottom[1]->num())
	  << "The data and label should have the same number.";*/
  vector<int> loss_shape(0);  // Loss layers output a scalar; 0 axes.
  top[0]->Reshape(loss_shape);
  
  data_dim = this->layer_param_.gmm_param().data_dim();
  class_size = this->layer_param_.gmm_param().class_size();
  int temp_sigma_min = this->layer_param_.gmm_param().sigma_min();
  int temp_sigma_max = this->layer_param_.gmm_param().sigma_max();
  sigma_min = exp((float)temp_sigma_min);
  sigma_max = exp((float)temp_sigma_max);
  if (temp_sigma_min >= temp_sigma_max){
	  sigma_min = -FLT_MAX;
	  sigma_max = FLT_MAX;
  }

  vector<int> diff_shape(3);		//[batchsize, class_size, data_dim] matrix
  diff_shape[0] = bottom[0]->shape()[0];
  diff_shape[1] = class_size;
  diff_shape[2] = data_dim;
  diff_.Reshape(diff_shape);
  diff_square_.Reshape(diff_shape);

  vector<int> class_dim(2);
  class_dim[0] = bottom[0]->shape()[0];
  class_dim[1] = class_size;
  diff_norm_.Reshape(class_dim);
  alpha_pi_.Reshape(class_dim);
  posterior_pi_.Reshape(class_dim);

  vector<int> batch_dim(1);
  batch_dim[0] = bottom[0]->shape()[0];
  alpha_pi_sum_.Reshape(batch_dim);
  batch_loss_.Reshape(batch_dim);
  sum_multiplier_.Reshape(batch_dim);

  Dtype* multiplier_data = sum_multiplier_.mutable_cpu_data();
  caffe_set(sum_multiplier_.count(), Dtype(1), multiplier_data);

  visualize_ = this->layer_param_.spatial_param().visualize();
}

template <typename Dtype>
void MDNLossLayer<Dtype>::Forward_cpu(const vector<Blob<Dtype>*>& bottom,
    const vector<Blob<Dtype>*>& top) {
  //CPU version not implemented
}

template <typename Dtype>
void MDNLossLayer<Dtype>::Backward_cpu(const vector<Blob<Dtype>*>& top,
    const vector<bool>& propagate_down, const vector<Blob<Dtype>*>& bottom) {
	//CPU version not implemented
}

#ifdef CPU_ONLY
STUB_GPU(MDNLossLayer);
#endif

INSTANTIATE_CLASS(MDNLossLayer);
REGISTER_LAYER_CLASS(MDNLoss);

}  // namespace caffe
