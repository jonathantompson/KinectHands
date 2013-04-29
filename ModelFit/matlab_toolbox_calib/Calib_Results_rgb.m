% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 536.155895306885670 ; 534.115605210934290 ];

%-- Principal point:
cc = [ 321.554093585524920 ; 255.145232440246640 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.026685762325105 ; -0.092003269250715 ; 0.011795601146576 ; 0.000203111106286 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 4.022613520007199 ; 4.156221124757987 ];

%-- Principal point uncertainty:
cc_error = [ 5.504946902883436 ; 5.107706774807770 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.017870993371741 ; 0.047420448933833 ; 0.002964014391257 ; 0.003426212748349 ; 0.000000000000000 ];

%-- Image size:
nx = 640;
ny = 480;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 19;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -2.258030e+00 ; -2.170013e+00 ; -3.873906e-02 ];
Tc_1  = [ -1.750223e+02 ; -1.087648e+02 ; 8.827558e+02 ];
omc_error_1 = [ 1.115817e-02 ; 1.123297e-02 ; 2.497892e-02 ];
Tc_error_1  = [ 9.112546e+00 ; 8.510406e+00 ; 7.672793e+00 ];

%-- Image #2:
omc_2 = [ 1.899703e+00 ; 1.917754e+00 ; 3.448378e-01 ];
Tc_2  = [ -1.717755e+02 ; -1.613545e+02 ; 8.380997e+02 ];
omc_error_2 = [ 8.784197e-03 ; 8.678057e-03 ; 1.544307e-02 ];
Tc_error_2  = [ 8.724067e+00 ; 8.062338e+00 ; 7.457125e+00 ];

%-- Image #3:
omc_3 = [ 1.698309e+00 ; 1.738057e+00 ; -3.220753e-01 ];
Tc_3  = [ -1.954899e+02 ; -9.358549e+01 ; 1.056277e+03 ];
omc_error_3 = [ 7.667193e-03 ; 9.443770e-03 ; 1.428902e-02 ];
Tc_error_3  = [ 1.081475e+01 ; 1.014647e+01 ; 8.240265e+00 ];

%-- Image #4:
omc_4 = [ -1.508744e+00 ; -2.473081e+00 ; 3.132587e-01 ];
Tc_4  = [ 4.864150e+01 ; -2.341226e+02 ; 9.500724e+02 ];
omc_error_4 = [ 8.172504e-03 ; 1.273231e-02 ; 2.126198e-02 ];
Tc_error_4  = [ 9.933274e+00 ; 9.056566e+00 ; 7.458002e+00 ];

%-- Image #5:
omc_5 = [ -1.626489e+00 ; -1.727495e+00 ; -1.937850e-01 ];
Tc_5  = [ -1.287753e+02 ; -2.225186e+02 ; 7.879906e+02 ];
omc_error_5 = [ 7.164034e-03 ; 8.152588e-03 ; 1.277248e-02 ];
Tc_error_5  = [ 8.238607e+00 ; 7.593092e+00 ; 6.420556e+00 ];

%-- Image #6:
omc_6 = [ -1.948619e+00 ; -1.832626e+00 ; -5.632865e-01 ];
Tc_6  = [ -2.055847e+02 ; -1.235797e+02 ; 7.129191e+02 ];
omc_error_6 = [ 7.092796e-03 ; 8.487240e-03 ; 1.423750e-02 ];
Tc_error_6  = [ 7.422954e+00 ; 6.944673e+00 ; 6.344875e+00 ];

%-- Image #7:
omc_7 = [ 1.915185e+00 ; 1.894596e+00 ; -3.250754e-01 ];
Tc_7  = [ -2.030603e+02 ; -7.576495e+01 ; 1.083649e+03 ];
omc_error_7 = [ 8.488095e-03 ; 1.007576e-02 ; 1.770252e-02 ];
Tc_error_7  = [ 1.108975e+01 ; 1.040844e+01 ; 8.477938e+00 ];

%-- Image #8:
omc_8 = [ 1.903557e+00 ; 2.012873e+00 ; 1.698904e-01 ];
Tc_8  = [ -3.693796e+02 ; -3.182947e+02 ; 9.172297e+02 ];
omc_error_8 = [ 8.439269e-03 ; 1.362475e-02 ; 1.915768e-02 ];
Tc_error_8  = [ 9.653547e+00 ; 9.123969e+00 ; 1.013015e+01 ];

%-- Image #9:
omc_9 = [ NaN ; NaN ; NaN ];
Tc_9  = [ NaN ; NaN ; NaN ];
omc_error_9 = [ NaN ; NaN ; NaN ];
Tc_error_9  = [ NaN ; NaN ; NaN ];

%-- Image #10:
omc_10 = [ -1.934932e+00 ; -2.014304e+00 ; -2.636318e-01 ];
Tc_10  = [ 2.778292e+00 ; -2.805395e+02 ; 7.822134e+02 ];
omc_error_10 = [ 8.932748e-03 ; 1.179802e-02 ; 1.716500e-02 ];
Tc_error_10  = [ 8.261199e+00 ; 7.694049e+00 ; 7.288627e+00 ];

%-- Image #11:
omc_11 = [ 2.232854e+00 ; 2.159795e+00 ; -2.357010e-01 ];
Tc_11  = [ -2.574945e+01 ; 2.109991e+01 ; 8.985536e+02 ];
omc_error_11 = [ 1.306587e-02 ; 9.344805e-03 ; 2.670525e-02 ];
Tc_error_11  = [ 9.218361e+00 ; 8.615352e+00 ; 7.526562e+00 ];

%-- Image #12:
omc_12 = [ 2.286918e+00 ; 2.100963e+00 ; -2.876664e-02 ];
Tc_12  = [ -4.972092e+02 ; 4.683463e+01 ; 9.564861e+02 ];
omc_error_12 = [ 1.353565e-02 ; 1.360900e-02 ; 2.522674e-02 ];
Tc_error_12  = [ 9.950760e+00 ; 9.843349e+00 ; 1.013705e+01 ];

%-- Image #13:
omc_13 = [ -9.982912e-01 ; -2.761407e+00 ; -6.976354e-02 ];
Tc_13  = [ -1.513955e+01 ; -2.079696e+02 ; 8.737768e+02 ];
omc_error_13 = [ 4.866057e-03 ; 1.266547e-02 ; 1.964530e-02 ];
Tc_error_13  = [ 9.064384e+00 ; 8.427816e+00 ; 7.372495e+00 ];

%-- Image #14:
omc_14 = [ 2.867999e+00 ; 1.251044e+00 ; -8.552970e-02 ];
Tc_14  = [ -2.490253e+02 ; 3.916761e+01 ; 8.051207e+02 ];
omc_error_14 = [ 1.131831e-02 ; 5.782526e-03 ; 2.274851e-02 ];
Tc_error_14  = [ 8.302139e+00 ; 7.763636e+00 ; 7.054452e+00 ];

%-- Image #15:
omc_15 = [ -2.016761e+00 ; -1.047845e+00 ; -6.035287e-01 ];
Tc_15  = [ -1.937385e+02 ; -1.941586e+00 ; 7.559938e+02 ];
omc_error_15 = [ 8.613260e-03 ; 7.879719e-03 ; 1.187574e-02 ];
Tc_error_15  = [ 7.775937e+00 ; 7.245745e+00 ; 6.100861e+00 ];

%-- Image #16:
omc_16 = [ -1.536465e+00 ; -1.318420e+00 ; -8.963377e-02 ];
Tc_16  = [ -1.653374e+02 ; -2.239566e+02 ; 9.124117e+02 ];
omc_error_16 = [ 8.272445e-03 ; 8.070895e-03 ; 1.104392e-02 ];
Tc_error_16  = [ 9.492721e+00 ; 8.766023e+00 ; 7.210385e+00 ];

%-- Image #17:
omc_17 = [ -1.423602e+00 ; -2.280882e+00 ; -1.266468e+00 ];
Tc_17  = [ 2.344269e+01 ; -9.789371e+01 ; 5.821985e+02 ];
omc_error_17 = [ 4.998994e-03 ; 1.136433e-02 ; 1.445931e-02 ];
Tc_error_17  = [ 6.019176e+00 ; 5.615195e+00 ; 5.785226e+00 ];

%-- Image #18:
omc_18 = [ 1.725205e+00 ; 2.154719e+00 ; -6.519094e-01 ];
Tc_18  = [ -6.618447e+01 ; -8.899411e+00 ; 1.017378e+03 ];
omc_error_18 = [ 7.609449e-03 ; 9.990851e-03 ; 1.772962e-02 ];
Tc_error_18  = [ 1.036690e+01 ; 9.749729e+00 ; 7.178975e+00 ];

%-- Image #19:
omc_19 = [ 1.909763e+00 ; 1.529352e+00 ; -3.528676e-01 ];
Tc_19  = [ -2.035724e+02 ; 3.414962e+01 ; 9.024845e+02 ];
omc_error_19 = [ 8.497642e-03 ; 8.118677e-03 ; 1.431569e-02 ];
Tc_error_19  = [ 9.229313e+00 ; 8.714934e+00 ; 7.157854e+00 ];

