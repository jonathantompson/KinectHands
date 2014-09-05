% Jonathan Tompson, New York University - 8/28/2014
% This is a simple script to visualize a training example
clearvars; close all; clc; rng(0);
addpath('.\export_fig\');

dataset_dir = '.\train\';
image_index = 1;
kinect_index = 1;
filename_prefix = sprintf('%d_%07d', kinect_index, image_index);

%% Load and display an RGB example
rgb = imread([dataset_dir, 'rgb_', filename_prefix, '.png']);
figure;
set(gcf, 'Position', [200 200 1800 400]);
subplot_tight(1,6,1);
imshow(rgb);

%% Load and display a synthetic depth example
% The top 8 bits of depth are packed into green and the lower 8 bits into blue.
synthdepth = imread([dataset_dir, 'synthdepth_', filename_prefix, '.png']);
synthdepth = uint16(synthdepth(:,:,3)) + bitsll(uint16(synthdepth(:,:,2)), 8);
subplot_tight(1,6,2);
ind = find(synthdepth > 0);
imshow(synthdepth, [min(synthdepth(ind))-10, max(synthdepth(ind))+10]);

%% Load and display a depth example
% The top 8 bits of depth are packed into green and the lower 8 bits into blue.
depth = imread([dataset_dir, 'depth_', filename_prefix, '.png']);
depth = uint16(depth(:,:,3)) + bitsll(uint16(depth(:,:,2)), 8);
subplot_tight(1,6,3);
imshow(depth, [0, max(depth(:))]);

%% Load the UVD Coefficients and display them on the depth image
load([dataset_dir, 'joint_data.mat']);
jnt_uvd = squeeze(joint_uvd(kinect_index, image_index, :, :));
jnt_colors = rand(size(jnt_uvd,1), 3);
hold on;
scatter(jnt_uvd(:,1), jnt_uvd(:,2), 20, jnt_colors, 'filled');

%% Visualize the depth in 3D
uvd = convert_depth_to_uvd(depth);
xyz = convert_uvd_to_xyz(uvd);
% Decimate the image (otherwise rendering will be too slow)
decimation = 4;
xyz_decimated = xyz(1:decimation:end, 1:decimation:end, :);
points = reshape(xyz_decimated, size(xyz_decimated,1)*size(xyz_decimated,2), 3);
% Collect the points within the AABBOX of the non-background points
body_points = points(find(points(:,3) < 2000),:);
axis_bounds = [min(body_points(:,1)) max(body_points(:,1)) ...
  min(body_points(:,3)) max(body_points(:,3)) ...
  min(body_points(:,2)) max(body_points(:,2))];
% Visualize the entire point cloud
subplot_tight(1,6,4,[0.05,0.05]);
plot3(body_points(:,1), body_points(:,3), body_points(:,2), '.', 'MarkerSize', 1.5);
axis(axis_bounds);
view(45,20);
set(gcf,'renderer','opengl'); axis vis3d; axis equal; hold on; grid on;
set(gca,'xticklabel',[]);
set(gca,'yticklabel',[]);
set(gca,'zticklabel',[]);

%% Visualize the hand and the joints in 3D
for i = 1:2
  subplot_tight(1,6,7-i,[0.05,0.05]);
  if i == 1
    uvd = convert_depth_to_uvd(depth);
  else
    uvd = convert_depth_to_uvd(synthdepth);
  end
  xyz = convert_uvd_to_xyz(uvd);
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
  plot3(points(:,1), points(:,3), points(:,2), '.', 'MarkerSize', 1.5); 
  set(gcf,'renderer','opengl'); axis vis3d; axis equal; hold on; grid on;
  scatter3(hand_points(:,1), hand_points(:,3), hand_points(:,2), 50, jnt_colors, 'Fill','LineWidth', 0.5);
  axis(axis_bounds);
  set(gca,'xticklabel',[]);
  set(gca,'yticklabel',[]);
  set(gca,'zticklabel',[]);
end

set(gcf, 'Color', [1 1 1]);
export_fig('sample.png', gcf, '-png', '-a4');
