clearvars; clc; close all;

filename = 'kinect_depth_image_uncompressed.bin';
src_x = 1;
src_y = 1;
src_width = 640;
src_height = 480;
src_stride = src_width;

dst_x = 1;
dst_y = 1;
dst_width = 320;
dst_height = 240;
dst_stride = dst_width;

upScaleX = src_width / dst_width;
upScaleY = src_height / dst_height;

depth = single(loadImageFile(filename, src_width, src_height, 1, 'int16'));
mag = 200;
% imshow((depth - min(min(depth))) / (max(max(depth)) - min(min(depth))), ...
%     'InitialMagnification', mag);

depth_down = downsample(depth, 2);
figure;
imshow((depth_down - min(min(depth_down))) / (max(max(depth_down)) - min(min(depth_down))), ...
    'InitialMagnification', mag);

filename = 'kinect_depth_image_uncompressed_down_float.bin';
depth_frac = single(loadImageFile(filename, dst_width, dst_height, 1, 'single'));
mag = 200;
figure;
imshow((depth_frac - min(min(depth_down))) / (max(max(depth_down)) - min(min(depth_down))), ...
    'InitialMagnification', mag);
