clear all; close all; clear global; clc;

filenames = ls('hands_*.bin');
filename =  filenames(1,:);  % Grab the first one
width = 640;
height = 480;
num_coeff = 26;

% Load in the raw data
file = fopen(filename);
depth = fread(file, width*height, 'int16');
label = fread(file, width*height, 'uint8');
coeff = fread(file, num_coeff, 'float');
fclose(file);

disp('Coeffs are:');
disp(coeff);

depth = single(reshape(depth, width, height )');
label = reshape(label, width, height )';

min_depth = min(min(depth(find(depth>0))));
max_depth = max(max(depth));

depth_scaled = zeros(height, width);
for v = 1:height
  for u = 1:width
    if (depth(v, u) ~= 0)
      depth_scaled(v, u) = (depth(v, u) - min_depth) / (max_depth - min_depth);
    end
  end
end

figure;
imshow(depth_scaled);

figure;
imshow(single(label) / 18);
