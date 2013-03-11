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
dmin = min(min(depth));
dmax = max(max(depth));
% imshow((depth - dmin) / (dmax - dmin), 'InitialMagnification', mag);

% depth_down = downsample(depth, 2);
% figure;
% imshow((depth_down - dmin) / (dmax - dmin), 'InitialMagnification', mag / 0.5);
% 
scale = 1 / 6.4;
w = 640 * scale;
h = 480 * scale;
filename = 'kinect_depth_image_uncompressed_down_float.bin';
depth_frac = single(loadImageFile(filename, w, h, 1, 'single'));
figure;
imshow((depth_frac - dmin) / (dmax - dmin), 'InitialMagnification', mag / scale);

depth_frac2 = imresize(depth, scale, 'box');
figure;
imshow((depth_frac2 - dmin) / (dmax - dmin), 'InitialMagnification', mag  / scale);

filename = 'kinect_depth_image_uncompressed_down_float2.bin';
depth_frac2 = single(loadImageFile(filename, w, h, 1, 'single'));
figure;
imshow((depth_frac2 - dmin) / (dmax - dmin), 'InitialMagnification', mag / scale);

filename = 'kinect_depth_image_uncompressed_up_float.bin';
depth_frac3 = single(loadImageFile(filename, 640/scale, 480/scale, 1, 'single'));
figure;
imshow((depth_frac3 - dmin) / (dmax - dmin), 'InitialMagnification', mag * scale);

depth_frac4 = imresize(depth, 1/scale, 'bilinear');
figure;
imshow((depth_frac4 - dmin) / (dmax - dmin), 'InitialMagnification', mag * scale);

% figure;
% delta = abs(depth_frac - depth_frac2);
% deltamin = min(min(delta));
% deltamax = max(max(delta))
% imshow((delta - deltamin) / (deltamax - deltamin), 'InitialMagnification', mag / scale);


% % Check mipmaps
% filename = 'kinect_depth_image_uncompressed_down_float.bin';
% mips = single(loadImageFile(filename, 640*480, 1, 1, 'single'));
% istart = 1;
% for i = 1:4
%   w = 640/2^i;
%   h = 480/2^i;
%   iend = istart + w*h - 1;
%   m = reshape(mips(istart:iend), 640/2^i, 480/2^i)';
%   figure;
%   imshow((m - dmin) / (dmax - dmin), 'InitialMagnification', mag * 2^i);
%   figure;
%   m_matlab = imresize(depth, 1/2^i, 'box');
%   imshow((m_matlab - dmin) / (dmax - dmin), 'InitialMagnification', mag * 2^i);
%   istart = iend + 1;
%   max(max(abs(m_matlab - m)))
% end

% % Check individual mipmaps
% filename = 'kinect_depth_image_uncompressed_down_float_l.bin';
% mip_l = single(loadImageFile(filename, 80, 60, 1, 'single'));
% figure;
% imshow((mip_l - dmin) / (dmax - dmin), 'InitialMagnification', mag * 640 / 80);
% filename = 'kinect_depth_image_uncompressed_down_float_u.bin';
% mip_u = single(loadImageFile(filename, 160, 120, 1, 'single'));
% figure;
% imshow((mip_u - dmin) / (dmax - dmin), 'InitialMagnification', mag * 640 / 160);
% figure;
% m_matlab = imresize(depth, 160 / 640, 'box');
% imshow((m_matlab - dmin) / (dmax - dmin), 'InitialMagnification', mag * 640 / 160);

