clear all; close all; clc; clear global;

% FYI: To increase the data cursor precision change:
% <path to matlab>\toolbox\matlab\graphics\@graphics\@datacursor\default_getDatatipText.m
% all the lines with "DEFAULT_DIGITS = 4;" to whatever you want

width = 640;
height = 480;
filenames = {'target_depth.bin', 'hand_depth_0.bin', 'hand_depth_1.bin', ...
  'hand_depth_2.bin', 'hand_depth_3.bin', 'hand_depth_4.bin', ...
  'hand_depth_5.bin', 'hand_depth_6.bin', 'hand_depth_7.bin'};

for i = 1:length(filenames)
  file = fopen(filenames{i});
  raw_depth = fread(file, width*height, 'float');
  fclose(file);

  min_val = min(raw_depth(find(raw_depth < (0.9999999))));
  max_val = max(raw_depth(find(raw_depth < (0.9999999))));
  target_depth = reshape(raw_depth, width, height )';

  target_depth = (target_depth - min_val) / (max_val - min_val);
  Argb = cat(3, target_depth, target_depth, target_depth);
  figure;
  imshow(Argb)
  title(filenames(i));
end