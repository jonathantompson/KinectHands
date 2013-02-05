clear all; close all; clear global; clc;

filename = './Hand.bin';
width = 640;
height = 480;

% Load in the raw data
file = fopen(filename);
xyz_data = fread(file, width*height*3, 'float');
hsv_data = fread(file, width*height*3, 'char');
mask_data = fread(file, width*height, 'char');  % 0-no hand, 1-LHand, 2-RHand, 3-both hands
fclose(file);
clear data;  % don't need it any more

% Represent raw double data as rgb
h = reshape(hsv_data(1:3:end-2), width, height )';
s = reshape(hsv_data(2:3:end-1), width, height )';
v = reshape(hsv_data(3:3:end), width, height )';
Ahsv = cat(3,h,s,v) / 255;
Argb = hsv2rgb(Ahsv);
figure;
imshow(Ahsv);
title('RAW HSV');

figure;
imshow(Argb);
title('RAW RGB');

figure;
imshow(h/255);
title('RAW Hue');

cutoff = 3000;
glove = zeros(height, width);
for y = 1:height
  for x = 1:width
    d_h = 255 - (h(y,x));
    d_s = 255 - (s(y,x));
    d_v = 255 - (v(y,x));
    glove(y,x) = d_h*d_h + d_s*d_s + d_v*d_v;
  end
end
figure;
imshow(medfilt2(glove < cutoff), [2 2]);
% figure;
% imshow(glove / 255);