clearvars; clc; close all;
cur_file = 'normal_0.bin';
depth_image = loadImageFile(cur_file, 640, 480, 3, 'single');
imshow(0.5 * (depth_image + 1), 'InitialMagnification', 200)
cur_file = 'normal_1.bin';
depth_image = loadImageFile(cur_file, 640, 480, 3, 'single');
imshow(0.5 * (depth_image + 1), 'InitialMagnification', 200)