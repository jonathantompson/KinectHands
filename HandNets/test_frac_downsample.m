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
depth = (depth - min(min(depth))) / (max(max(depth)) - min(min(depth)));
mag = 200;
imshow(depth, 'InitialMagnification', mag);

int_image = integral_image(depth, src_width, src_height);

for x = dst_x:(dst_x + dst_w - 1)
  s1 = (x - src_x) * upScaleX + src_x;
  sr = (x + 1 - dx)
end

imshow((int_image - min(min(int_image))) / (max(max(int_image)) - min(min(int_image))));

filename = 'kinect_depth_image_uncompressed_down_float.bin';
depth = single(loadImageFile(filename, dst_width, dst_height, 1, 'single'));
depth = (depth - min(min(depth))) / (max(max(depth)) - min(min(depth)));
mag = 200;
imshow(depth, 'InitialMagnification', mag);
