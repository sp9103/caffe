#include <vector>

#include "caffe/layer.hpp"
#include "caffe/util/io.hpp"
#include "caffe/util/math_functions.hpp"
#include "caffe/layers/mdn_loss_layer.hpp"

#include <opencv2\opencv.hpp>

#define MATH_PI		3.14159265

namespace caffe {

template <typename Dtype>							//mu_ik - tk calculation
__global__ void kernel_label_subtract(const int count,
	const int param_size, const int class_size, const int data_dim,
	const Dtype* data, const Dtype* label, Dtype* diff) {
	CUDA_KERNEL_LOOP(index, count) {
		int internal_idx = index % data_dim;					//mu vector에서 몇번째 인덱스
		int outer_idx = index / data_dim;						//몇번째 클래스
		int label_idx = index / (class_size * data_dim);		//몇번째 label == 몇번째 batch
		diff[index] = data[outer_idx * param_size + internal_idx + 1] - label[label_idx * data_dim + internal_idx];
	}
}

template <typename Dtype>							// || mu-t || ^ 2
__global__ void kernel_diff_norm(const int count,
	const int class_size, const int data_dim,
	const Dtype* diff_squre, Dtype* norm) {
	CUDA_KERNEL_LOOP(index, count) {
		Dtype sum = 0;
		for (int i = 0; i < data_dim; i++)
			sum += diff_squre[index * data_dim + i];
		norm[index] = sum;
	}
}

template <typename Dtype>							// alpha * gaussian distribution 계산
__global__ void kernel_normal_distribution(const int count,
	const int param_size, const int class_size, const int data_dim,
	const Dtype* norm, const Dtype* data, Dtype* alpha_distribution) {
	CUDA_KERNEL_LOOP(index, count) {
		Dtype alpha = data[index*param_size];
		Dtype sigma = data[index*param_size + 1 + data_dim];
		Dtype sigma_9 = pow(sigma, data_dim);
		Dtype exp_gaussian = exp(- norm[index] / sigma / sigma / 2);
		Dtype distribution = 0;
		if (exp_gaussian != 0)
			distribution = exp_gaussian / pow(sigma, data_dim) / pow(2 * MATH_PI, data_dim / 2);
		//alpha * gaussian_distribution;
		alpha_distribution[index] = alpha * distribution;
	}
}

template <typename Dtype>							// ∑(alpha * gaussian distribution) 계산
__global__ void kernel_class_summation(const int count, const int class_size,
	const Dtype* alpha_pi_, Dtype* alpha_pi_sum_) {
	CUDA_KERNEL_LOOP(index, count) {
		Dtype sum = 0;
		for (int i = 0; i < class_size; i++)
			sum += alpha_pi_[index * class_size + i];
		if (sum != 0)
			alpha_pi_sum_[index] = sum;
	}
}

template <typename Dtype>							// posterior calculation 
__global__ void kernel_posterior_calc(const int count,
	const int batch_size, const int class_size,
	const Dtype* alpha_pi_, const Dtype* alpha_pi_sum_, Dtype* posterior) {
	CUDA_KERNEL_LOOP(index, count) {
		const int batch_idx = index / class_size;
		posterior[index] = alpha_pi_[index] / alpha_pi_sum_[batch_idx];
	}
}

template <typename Dtype>							// backpropagation delta calculation 
__global__ void kernel_delta_calc(const int count,
	const int batch_size, const int class_size, const int param_size, const int data_dim, float sigma_min, float sigma_max,
 	const Dtype* posterior, const Dtype* diff, const Dtype* diff_norm, const Dtype* bottom_data, Dtype* bottom_diff) {
	CUDA_KERNEL_LOOP(index, count) {
 		const int internal_idx = index % param_size;
		const int class_idx = index / param_size;
		const Dtype sigma = bottom_data[class_idx*param_size + param_size - 1];
		if (internal_idx == 0){							//alpha delta calculate
			bottom_diff[index] = bottom_data[index] - posterior[class_idx];
		}
		else if (internal_idx == param_size - 1){		//sigma delta calculate
			if (sigma_min > sigma || sigma_max < sigma)		bottom_diff[index] = 0;
			else
				bottom_diff[index] = -posterior[class_idx] * (diff_norm[class_idx] / sigma / sigma - data_dim);
		}
		else{											//mu delta calculate
			const int data_idx = internal_idx - 1;		//[0, datadim-1]
			Dtype diff_ik = diff[data_dim * class_idx + data_idx];
			bottom_diff[index] = posterior[class_idx] * (diff_ik / sigma / sigma);
		}
	}
}

//template <typename Dtype>
//__global__ void kernel_gradient_cliping(const int count,
//	const int batch_size, const int class_size, const int class_dim,
//	const Dtype threshold, const Dtype* norm, Dtype* gradient ){
//	CUDA_KERNEL_LOOP(index, count) {
//		const int batchIdx = index % (class_size * class_dim);
//
//		if (norm[batchIdx] > threshold){
//			gradient[index] = threshold / norm[index] * gradient[index];
//		}
//	}
//}
//
//template <typename Dtype>
//__global__ void kernel_gradient_norm(const int count,
//	const int batch_size, const int class_size, const int class_dim,
//	const Dtype *squre, Dtype *norm){
//	CUDA_KERNEL_LOOP(index, count) {
//		const int batchidx = index;
//
//		Dtype sum = 0;
//		for (int i = 0; i < class_dim * class_size; i++){
//			sum += squre[batchidx * (class_dim * class_size) + i];
//		}
//		norm[batchidx] = sqrt(sum);
//	}
//}

template <typename Dtype>
__global__ void kernel_inexp_x(const int count,
	const int param_size, const int class_size, const int data_dim,
	const Dtype* norm, const Dtype* data, Dtype* alpha_pi_x){
	CUDA_KERNEL_LOOP(index, count) {
		const Dtype epsilon = 0.00001;
		Dtype alpha = data[index*param_size];
		Dtype sigma = data[index*param_size + 1 + data_dim];
		alpha_pi_x[index] = log(alpha + epsilon) - data_dim / 2 * log(2 * MATH_PI*sigma + epsilon) - norm[index] / 2 / sigma;
	}
}

template <typename Dtype>
__global__ void kernel_max_inexp(const int count, 
	const int batch_size, const int class_size,
	const Dtype* alpha_pi_x, Dtype* max){
	CUDA_KERNEL_LOOP(index, count) {
		max[index] = -FLT_MAX;
		for (int i = 0; i < class_size; i++){
			if (max[index] < alpha_pi_x[index*class_size + i])
				max[index] = alpha_pi_x[index*class_size + i];
		}
	}
}

template <typename Dtype>
__global__ void kernel_sub_inexp_m(const int count,
	const int batch_size, const int class_size,
	const Dtype * max, const Dtype *x,
	Dtype *x_m, Dtype *alpha_pi_x){
	CUDA_KERNEL_LOOP(index, count) {
		int batch_idx = index / class_size;
		x_m[index] = x[index] - max[batch_idx];
		alpha_pi_x[index] = exp(x_m[index]);
	}
}

template <typename Dtype>
__global__ void kernel_calc_inexp_loss(const int count,
	const int batch_size, const int class_size,
	const Dtype *max, const Dtype *alpha_pi_x, Dtype *alpha_pi_x_sum, Dtype *batch_loss){
	CUDA_KERNEL_LOOP(index, count) {
		alpha_pi_x_sum[index] = 0;	//∑e^(x-m)
		for (int i = 0; i < 5; i++){
			alpha_pi_x_sum[index] += alpha_pi_x[class_size * index + i];
		}
		batch_loss[index] = max[index] + log(alpha_pi_x_sum[index]);
	}
}

template <typename Dtype>
void MDNLossLayer<Dtype>::Forward_gpu(const vector<Blob<Dtype>*>& bottom,
    const vector<Blob<Dtype>*>& top) {

	const Dtype* bottom_data = bottom[0]->gpu_data();
	const Dtype* label = bottom[1]->gpu_data();
	const int batch_size = bottom[0]->shape()[0];

	//subtract (mu - t)
	kernel_label_subtract<Dtype> << <CAFFE_GET_BLOCKS(diff_.count()), CAFFE_CUDA_NUM_THREADS >> >(diff_.count(),
		data_dim + 2, class_size, data_dim, bottom_data, label, diff_.mutable_gpu_data());

	//square ( mu - t )^2
	caffe_gpu_mul(diff_.count(), diff_.gpu_data(), diff_.gpu_data(), diff_square_.mutable_gpu_data());

	//norm  : || mu-t || ^ 2
	kernel_diff_norm<Dtype> << <CAFFE_GET_BLOCKS(class_size * batch_size), CAFFE_CUDA_NUM_THREADS >> >(class_size * batch_size,
		class_size, data_dim, diff_square_.gpu_data(), diff_norm_.mutable_gpu_data());

	//calculate gaussian distribution
	//kernel_normal_distribution<Dtype> << <CAFFE_GET_BLOCKS(class_size * batch_size), CAFFE_CUDA_NUM_THREADS >> >(class_size * batch_size,
	//	data_dim + 2, class_size, data_dim,
	//	diff_norm_.gpu_data(), bottom_data, alpha_pi_.mutable_gpu_data());

	//find max_alpha_pi
	//calc x
	kernel_inexp_x<Dtype> << <CAFFE_GET_BLOCKS(class_size * batch_size), CAFFE_CUDA_NUM_THREADS >> >(class_size * batch_size,
		data_dim + 2, class_size, data_dim,
		diff_norm_.gpu_data(), bottom_data, x_.mutable_gpu_data());
	//find max
	kernel_max_inexp<Dtype> << <CAFFE_GET_BLOCKS(batch_size), CAFFE_CUDA_NUM_THREADS >> >(batch_size,
		batch_size, class_size,
		x_.gpu_data(), max_alpha_pi_.mutable_gpu_data());
	//sub exp(x-m)
	kernel_sub_inexp_m<Dtype> << <CAFFE_GET_BLOCKS(class_size * batch_size), CAFFE_CUDA_NUM_THREADS >> >(class_size * batch_size,
		batch_size, class_size,
		max_alpha_pi_.gpu_data(), x_.gpu_data(),
		x_m_.mutable_gpu_data(), alpha_pi_.mutable_gpu_data());
	//calc loss
	kernel_calc_inexp_loss<Dtype> << <CAFFE_GET_BLOCKS(batch_size), CAFFE_CUDA_NUM_THREADS >> >(batch_size,
		batch_size, class_size,
		max_alpha_pi_.gpu_data(), alpha_pi_.gpu_data(), alpha_pi_sum_.mutable_gpu_data(), batch_loss_.mutable_gpu_data());

	//sumation : ∑(alpha * distribution)
	//kernel_class_summation<Dtype> << <CAFFE_GET_BLOCKS(batch_size), CAFFE_CUDA_NUM_THREADS >> >(batch_size, class_size, alpha_pi_.gpu_data(), alpha_pi_sum_.mutable_gpu_data());

	//loss : ln ( sumation ) / number of batchsize
	Dtype loss;
	//caffe_gpu_log(alpha_pi_sum_.count(), alpha_pi_sum_.gpu_data(), batch_loss_.mutable_gpu_data());
	caffe_gpu_dot(batch_loss_.count(), batch_loss_.gpu_data(), sum_multiplier_.gpu_data(), &loss);
	loss /= bottom[0]->num();
	top[0]->mutable_cpu_data()[0] = -loss;

	if (visualize_){
		//write file
		FILE *fp = fopen("loss_output.txt", "w");

		Dtype *out = new Dtype[class_size * 11];
		Dtype label_out[9];
		cudaMemcpy(out, bottom[0]->gpu_data(), sizeof(Dtype) * class_size * 11, cudaMemcpyDeviceToHost);
		cudaMemcpy(label_out, bottom[1]->gpu_data(), sizeof(Dtype) * 9, cudaMemcpyDeviceToHost);

		for (int i = 0; i < 9; i++)
			fprintf(fp, "%f ", label_out[i]);
		fprintf(fp, "\n");

		for (int i = 0; i < class_size * 11; i++)
			fprintf(fp, "%f ", out[i]);

		delete[] out;

		fclose(fp);
	}

	if (std::isnan(loss) || std::isinf(loss) /*|| loop_count % 100 == 0*/){
		printf("loss invalid value.\n");
		loop_count = 0;

		Dtype norm_box[5];
		Dtype diff_box[45], diff_squre_box[45];
		Dtype bot_box[55], label_box[9];
		Dtype dist_box[5], m_box;
		Dtype x_box[5], x_m_box[5];
		Dtype norm;
		Dtype alpha_pi_sum__box, alpha_pi_sum__box_temp;
		Dtype lossslice;

		for (int i = 0; i < batch_size; i++){
			cudaMemcpy(diff_box, &diff_.gpu_data()[i * 45], sizeof(Dtype) * 45, cudaMemcpyDeviceToHost);
			cudaMemcpy(label_box, &label[i * 9], sizeof(Dtype) * 9, cudaMemcpyDeviceToHost);
			cudaMemcpy(bot_box, &bottom_data[55 * i], sizeof(Dtype) * 55, cudaMemcpyDeviceToHost);
			cudaMemcpy(diff_squre_box, &diff_square_.gpu_data()[i * 45], sizeof(Dtype) * 45, cudaMemcpyDeviceToHost);
			cudaMemcpy(norm_box, &diff_norm_.gpu_data()[i * 5], sizeof(Dtype) * 5, cudaMemcpyDeviceToHost);
			cudaMemcpy(dist_box, &alpha_pi_.gpu_data()[i * 5], sizeof(Dtype) * 5, cudaMemcpyDeviceToHost);
			cudaMemcpy(&m_box, &max_alpha_pi_.gpu_data()[i], sizeof(Dtype), cudaMemcpyDeviceToHost);
			cudaMemcpy(x_box, &x_.gpu_data()[i * 5], sizeof(Dtype) * 5, cudaMemcpyDeviceToHost);
			cudaMemcpy(x_m_box, &x_m_.gpu_data()[i * 5], sizeof(Dtype) * 5, cudaMemcpyDeviceToHost);
			cudaMemcpy(&alpha_pi_sum__box, &alpha_pi_sum_.gpu_data()[i], sizeof(Dtype), cudaMemcpyDeviceToHost);
			cudaMemcpy(&lossslice, &batch_loss_.gpu_data()[i], sizeof(Dtype), cudaMemcpyDeviceToHost);

			if (std::isnan(lossslice) || std::isinf(lossslice)){
				printf("slice of loss overflow\n");

				Dtype TotalX[5 * 128];
				cudaMemcpy(TotalX, x_.gpu_data(), sizeof(Dtype) * 128 * 5, cudaMemcpyDeviceToHost);
				for (int index = 0; index < 128; index++){
					Dtype tempmax = -FLT_MAX;
					for (int i = 0; i < class_size; i++){
						if (tempmax < TotalX[index*class_size + i])
							tempmax = TotalX[index*class_size + i];
					}
				}
			}
		}
	}

	//loop_count++;
}

//Diff 0번지는 값있고 1번지는 없음
template <typename Dtype>
void MDNLossLayer<Dtype>::Backward_gpu(const vector<Blob<Dtype>*>& top,
	const vector<bool>& propagate_down, const vector<Blob<Dtype>*>& bottom) {
	for (int i = 0; i < 2; ++i) {
		if (propagate_down[i]) {
			// i == 0 : bottom network i == 1 : label
			//부호 +- 다시 한번 생각해보기
			Dtype* bottom_diff = bottom[i]->mutable_gpu_diff();
			const Dtype* bottom_data = bottom[i]->gpu_data();
			const int batch_size = bottom[0]->shape()[0];
			
			//calculate posterior probability ( alpha*pi / sumation ( alpha_i * pi_i )
			kernel_posterior_calc<Dtype> << <CAFFE_GET_BLOCKS(batch_size*class_size), CAFFE_CUDA_NUM_THREADS >> >
				(batch_size*class_size, batch_size, class_size, 
				alpha_pi_.gpu_data(), alpha_pi_sum_.gpu_data(), posterior_pi_.mutable_gpu_data());

			//calculate bottom diff (alpha_diff, mu_diff, sigma_diff)
			kernel_delta_calc<Dtype> << <CAFFE_GET_BLOCKS(bottom[i]->count()), CAFFE_CUDA_NUM_THREADS >> >(bottom[i]->count(),
				batch_size, class_size, data_dim + 2, data_dim, sigma_min, sigma_max,
				posterior_pi_.gpu_data(), diff_.gpu_data(), diff_norm_.gpu_data(), bottom_data, bottom_diff);

			//Dtype gradTemp[55];
			//for (int j = 0; j < batch_size; j++){
			//	cudaMemcpy(gradTemp, &bottom[i]->mutable_gpu_diff()[j * 55], sizeof(Dtype) * 55, cudaMemcpyDeviceToHost);

			//}
			
		}
	}
}

INSTANTIATE_LAYER_GPU_FUNCS(MDNLossLayer);

}  // namespace caffe
