#ifndef CAFFE_EUCLIDEAN_LOSS_LAYER_HPP_
#define CAFFE_EUCLIDEAN_LOSS_LAYER_HPP_

#include <vector>

#include "caffe/blob.hpp"
#include "caffe/layer.hpp"
#include "caffe/proto/caffe.pb.h"

#include "caffe/layers/loss_layer.hpp"

namespace caffe {

	template <typename Dtype>
	class MDNLossLayer : public LossLayer<Dtype> {
	public:
		explicit MDNLossLayer(const LayerParameter& param)
			: LossLayer<Dtype>(param) {}
		virtual void Reshape(const vector<Blob<Dtype>*>& bottom,
			const vector<Blob<Dtype>*>& top);

		virtual inline const char* type() const { return "MDNLoss"; }

		virtual inline bool AllowForceBackward(const int bottom_index) const {
			return true;
		}

	protected:
		virtual void Forward_cpu(const vector<Blob<Dtype>*>& bottom,
			const vector<Blob<Dtype>*>& top);
		virtual void Forward_gpu(const vector<Blob<Dtype>*>& bottom,
			const vector<Blob<Dtype>*>& top);

		virtual void Backward_cpu(const vector<Blob<Dtype>*>& top,
			const vector<bool>& propagate_down, const vector<Blob<Dtype>*>& bottom);
		virtual void Backward_gpu(const vector<Blob<Dtype>*>& top,
			const vector<bool>& propagate_down, const vector<Blob<Dtype>*>& bottom);

		int class_size;
		int data_dim;
		float sigma_min;
		float sigma_max;
		bool visualize_;

		int loop_count = 0;
		
		//Dtype grad_clip;z
		//int log_limit = 0;
		//Dtype total = 0;

		Blob<Dtype> diff_;					//(mu-t)
		Blob<Dtype> diff_square_;			//(mu-t)^2
		Blob<Dtype> diff_norm_;				//|| mu-t || ^ 2
		Blob<Dtype> alpha_pi_;				//gaussian distribution -> alpha*distribution - > pi( alpha*distribution / alpha_pi_sum_ )
		Blob<Dtype> alpha_pi_sum_;			//alpha_0 * distribution_0 + ... + alpha_m * distribution_m
		Blob<Dtype> batch_loss_;			//each batch loss
		Blob<Dtype> sum_multiplier_;		//dot product summation
		Blob<Dtype> posterior_pi_;			//posterior probability
		Blob<Dtype> max_alpha_pi_;
		Blob<Dtype> x_m_;
		Blob<Dtype> x_;

		//Blob<Dtype> grad_norm;				//gradient_norm(debug)
	};

}  // namespace caffe

#endif  // CAFFE_EUCLIDEAN_LOSS_LAYER_HPP_
