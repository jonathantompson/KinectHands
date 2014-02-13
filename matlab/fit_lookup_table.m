clearvars; clc; close all;

datafile = '../depth_lookup_table.bin';
depth_w = 512;
depth_h = 424;
depth_dim = depth_w * depth_h;
depth_hfov = 70.6;
depth_vfov = 60.0;
lookup2d = loadImageFile(datafile, depth_w, depth_h, 2, 'float32');
% lookup2d is size height x width x 2
x = squeeze(lookup2d(:,:,1));  % height x width
y = squeeze(lookup2d(:,:,2));  % height x width
u = repmat(0:depth_w-1, depth_h, 1);
v = repmat((0:depth_h-1)', 1, depth_w);
d = zeros(size(x));
uvd = cat(3,u,v,d);



% The lookup table is actually a 3D ray vector of unit lenght (so we can
% recover the full 3D ray within a sign change.
z = ones(size(x));
xyz = cat(3,x,y,z);
% Normalize the ray vectors
len = sqrt(sum(xyz.^2, 3));
xyz = xyz ./ repmat(len, 1, 1, 3);

s = 50;  % skip
quiver3(uvd(1:s:end,1:s:end,1), uvd(1:s:end,1:s:end,2), ...
  uvd(1:s:end,1:s:end,3), xyz(1:s:end,1:s:end,1), xyz(1:s:end,1:s:end,2),...
  xyz(1:s:end,1:s:end,3), 0.5);