% Jonathan Tompson, New York University - 8/28/2014
% This is a simple example script to visualize an example
clearvars; close all; clc; rng(0);

dataset_dir = '../data/hand_depth_data_processed_for_CN';
image_index = 0;
kinect_index = 1;
filename_prefix = sprintf('%s/kinect%d_image%07d', dataset_dir, ...
  kinect_index, image_index);

%% Load and display an RGB example
rgb = imread([filename_prefix, '_rgb.png']);
figure;
imshow(rgb);

%% Load and display a depth example
% The top 8 bits of depth are packed into green and the lower 8 bits into blue.
depth = imread([filename_prefix, '_depth.png']);
depth = uint16(depth(:,:,3)) + bitsll(uint16(depth(:,:,2)), 8);
figure;
imshow(depth, [0, max(depth(:))]);

%% Load the UVD Coefficients and display them on the depth image
joints = {'PALM_3','PALM_1','PALM_2','TH_KNU3_A','TH_KNU3_B','TH_KNU2_B',...
  'F1_KNU3_A','F1_KNU2_B','F2_KNU3_A','F2_KNU2_B','F3_KNU3_A','F3_KNU2_B',...
  'F4_KNU3_A','F4_KNU2_B'};
fid = fopen([filename_prefix, '_uvd.bin'], 'r'); 
[jnt_data, count] = fread(fid, 'float');
fclose(fid);
assert(count == length(joints) * 3);  % Just in case
jnt_data = reshape(jnt_data, 3, length(joints))';
jnt_colors = rand(length(joints), 3);
hold on;
scatter(jnt_data(:,1), jnt_data(:,2), 50, jnt_colors, 'filled');

%% Visualize the depth in 3D
uvd = convert_depth_to_uvd(depth);
xyz = convert_uvd_to_xyz(uvd);
% Decimate the image (otherwise rendering will be too slow)
decimation = 20;
xyz = xyz(1:decimation:end, 1:decimation:end, :);  
colors = double(rgb(1:decimation:end, 1:decimation:end, :)) / 255;
points = reshape(xyz, size(xyz,1)*size(xyz,2), 3);
colors = reshape(colors, size(colors,1)*size(colors,2), 3);
figure;
scatter3(points(:,1), points(:,3), points(:,2), 10, colors, 'Fill','LineWidth', 0.5);
set(gcf,'renderer','opengl'); axis vis3d; axis equal; hold on;
% Visualize the joints
xyz = squeeze(convert_uvd_to_xyz(reshape(jnt_data, 1, length(joints), 3)));
scatter3(xyz(:,1), xyz(:,3), xyz(:,2), 10, jnt_colors, 'Fill','LineWidth', 0.5);
