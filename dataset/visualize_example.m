% Jonathan Tompson, New York University - 8/28/2014
% This is a simple example script to visualize an example
clearvars; close all; clc; rng(0);

dataset_dir = '../data/dataset/';
image_index = 10;
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
load([dataset_dir, 'joint_data.mat']);
jnt_uvd = squeeze(joint_uvd(kinect_index, image_index, :, :));
jnt_colors = rand(size(jnt_uvd,1), 3);
hold on;
scatter(jnt_uvd(:,1), jnt_uvd(:,2), 50, jnt_colors, 'filled');

%% Visualize the depth in 3D
uvd = convert_depth_to_uvd(depth);
xyz = convert_uvd_to_xyz(uvd);
% Decimate the image (otherwise rendering will be too slow)
decimation = 5;
xyz_decimated = xyz(1:decimation:end, 1:decimation:end, :);
points = reshape(xyz_decimated, size(xyz_decimated,1)*size(xyz_decimated,2), 3);
% Visualize the entire point cloud
figure;
set(gcf, 'Position', [200 200 800 600]);
plot3(points(:,1), points(:,3), points(:,2), '.');
set(gcf,'renderer','opengl'); axis vis3d; axis equal; hold on;

%% Visualize the hand and the joints in 3D
points = reshape(xyz, size(xyz,1)*size(xyz,2), 3);
colors = reshape(rgb, size(rgb,1)*size(rgb,2), 3);
hand_points = squeeze(convert_uvd_to_xyz(reshape(jnt_uvd, 1, size(jnt_uvd,1), 3)));
% Collect the points within the AABBOX of the hand
axis_bounds = [min(hand_points(:,1)) max(hand_points(:,1)) ...
  min(hand_points(:,3)) max(hand_points(:,3)) ...
  min(hand_points(:,2)) max(hand_points(:,2))];
axis_bounds([1 3 5]) = axis_bounds([1 3 5]) - 20;
axis_bounds([2 4 6]) = axis_bounds([2 4 6]) + 20;
ipnts = find(points(:,1) >= axis_bounds(1) & points(:,1) <= axis_bounds(2) & ...
  points(:,2) >= axis_bounds(5) & points(:,2) <= axis_bounds(6) & ...
  points(:,3) >= axis_bounds(3) & points(:,3) <= axis_bounds(4));
points = points(ipnts, :);
colors = double(colors(ipnts, :))/255;
figure;
set(gcf, 'Position', [200 200 800 600]);
plot3(points(:,1), points(:,3), points(:,2), '.', 'LineWidth', 0.5); 
set(gcf,'renderer','opengl'); axis vis3d; axis equal; hold on; grid on;
scatter3(hand_points(:,1), hand_points(:,3), hand_points(:,2), 50, jnt_colors, 'Fill','LineWidth', 0.5);
axis(axis_bounds);
