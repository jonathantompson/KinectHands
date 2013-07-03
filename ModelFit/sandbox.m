clear all; close all; clc; clear global;

format longe;

lambda_k = 0.01;
HAND_NUM_COEFF = 8;
coeff_k_ = [1.0, -0.25, 0, 10, 0.24740396440029144, 0, 0, 0.96891242265701294];

jacobian_k_ = [-0.00034198936020501947, -0.0016309451447362733,...
  5.7459659714709233e-6, -8.5171992552091069e-6,  -3.5660824693195536e-7...
  -0.00010359741314402982, -9.9112534424961041e-5, -3.8268971991328726e-8];
jacobian_k_tran_ = jacobian_k_';

r_k = 0.40962301988231786;

delta_y = r_k;

delta_y_prime_ = jacobian_k_tran_ * delta_y;

normal_mat_ = jacobian_k_tran_ * jacobian_k_;

for i = 1:HAND_NUM_COEFF
  normal_mat_(i, i) = normal_mat_(i, i) + (lambda_k * normal_mat_(i, i));
end

delta_c_k_p1_ = normal_mat_ \ delta_y_prime_;  %% Full LU decomp using back-slash operator

coeff_k_p1_ = coeff_k_ + delta_c_k_p1_';
len = sqrt(coeff_k_p1_(5) * coeff_k_p1_(5) + coeff_k_p1_(6) * coeff_k_p1_(6) + ...
  coeff_k_p1_(7) * coeff_k_p1_(7) + coeff_k_p1_(8) * coeff_k_p1_(8));
coeff_k_p1_(5:8) = coeff_k_p1_(5:8) ./ len;

