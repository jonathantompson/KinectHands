clear all; close all; clear global; clc;

filename = [getenv('HOME'), '/Desktop/Hand.bin'];
width = 640;
height = 480;

% Load in the raw data
file = fopen(filename);
xyz_data = fread(file, width*height*3, 'float');
rgb_data = fread(file, width*height*3, 'char');
mask_data = fread(file, width*height, 'char');  % 0-no hand, 1-LHand, 2-RHand, 3-both hands
fclose(file);
clear data;  % don't need it any more

% Check the depth data
x = reshape(xyz_data(1:3:end-2), width, height )';
y = reshape(xyz_data(2:3:end-1), width, height )';
z = reshape(xyz_data(3:3:end), width, height )';
mask_data = reshape(mask_data, width, height )';
lHand_ind = find(bitand(mask_data,1)~=0);
rHand_ind = find(bitand(mask_data,2)~=0);
lHand_ind_nan = find(bitand(mask_data,1)==0);
rHand_ind_nan = find(bitand(mask_data,2)==0);
figure;
scatter3(x(lHand_ind), y(lHand_ind), z(lHand_ind));
figure;
scatter3(x(rHand_ind), y(rHand_ind), z(rHand_ind));

% Represent raw double data as rgb
r = reshape(rgb_data(1:3:end-2), width, height )';
g = reshape(rgb_data(2:3:end-1), width, height )';
b = reshape(rgb_data(3:3:end), width, height )';
Argb = cat(3,r,g,b);
figure;
imshow(Argb/255)

% Now see if the mapping between depth and RGB is correct:
r_lHand_masked = r;  
g_lHand_masked = g;  
b_lHand_masked = b;
r_lHand_masked(lHand_ind_nan) = 1;
g_lHand_masked(lHand_ind_nan) = 1;
b_lHand_masked(lHand_ind_nan) = 1;
Argb_masked = cat(3,r_lHand_masked,g_lHand_masked,b_lHand_masked);
figure;
imshow(Argb_masked/255)

r_rHand_masked = r;  
g_rHand_masked = g;  
b_rHand_masked = b;  
r_rHand_masked(rHand_ind_nan) = 1;
g_rHand_masked(rHand_ind_nan) = 1;
b_rHand_masked(rHand_ind_nan) = 1;
Argb_masked = cat(3,r_rHand_masked,g_rHand_masked,b_rHand_masked);
figure;
imshow(Argb_masked/255)