clearvars; clc; close all;
cur_file = 'residue_texture.bin';
depth_image = loadImageFile(cur_file, 640, 480, 1, 'single');
imshow(depth_image/30, 'InitialMagnification', 200)