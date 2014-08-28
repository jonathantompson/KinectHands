% Jonathan Tompson, New York University - 8/28/2014
% This is a simple example script to visualize an example
clearvars; close all; clc;
rng(0);

dataset_dir = '../data/hand_depth_data_processed_for_CN';
image_index = 0;
kinect_index = 0;

% Load and display an RGB example
rgb = imread(sprintf('%s/kinect%d_image%07d_rgb.png', dataset_dir, ...
  image_index, kinect_index));
figure;
imshow(rgb);

% Load and display a depth example
% The top 8 bits of depth are packed into green and the lower 8 bits into blue.
depth = imread(sprintf('%s/kinect%d_image%07d_depth.png', dataset_dir, ...
  image_index, kinect_index));
depth = uint16(depth(:,:,3)) + bitsll(uint16(depth(:,:,2)), 8);
figure;
imshow(double(depth) / double(max(depth(:))));

% Load the UVD Coefficients
joints = {'PALM_3','PALM_1','PALM_2','TH_KNU3_A','TH_KNU3_B','TH_KNU2_B',...
  'F1_KNU3_A','F1_KNU2_B','F2_KNU3_A','F2_KNU2_B','F3_KNU3_A','F3_KNU2_B',...
  'F4_KNU3_A','F4_KNU2_B'};
fid = fopen(sprintf('%s/kinect%d_image%07d_uvd.bin', dataset_dir, ...
  image_index, kinect_index), 'r'); 
[data, count] = fread(fid, 'float');
fclose(fid);
assert(count == length(joints) * 3);  % Just in case
data = reshape(data, 3, length(joints));
colors = rand(length(joints), 3);
hold on;
scatter(data(1,:)', data(2,:)', 50, colors, 'filled');