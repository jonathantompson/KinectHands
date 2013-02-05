clear all; close all; clear global; clc;

global im_width;   im_width = 640;
global im_height;  im_height = 480;

% try loading a file
data_dir = './hand_depth_data';
disp('Loading hands from directory...');
DB = loadHandDepthDataFromDirectory(data_dir, 1e5, 0);
% DB = loadFakeHandDepthDataFromDirectory(100, 0);
disp('Finished loading hands from directory...');
n_samples = length(DB.image_data(1,:));
single_plot_index = 245;
disp(['plotting file ', DB.filenames{single_plot_index}]);
depth_data = DB.image_data(:,single_plot_index);
mask_data = DB.label_data(:,single_plot_index);

% % plot the mask
im_mask = reshape(mask_data, im_width, im_height)';
im_mask_neg = uint16(~im_mask);
% Argb = cat(3, im_mask, im_mask, im_mask);
% figure;
% imshow(Argb.*(2^16-1));

% plot the depth
im_depth_data = uint16(reshape(depth_data, im_width, im_height)');
Argb = cat(3, im_depth_data, im_depth_data.*im_mask_neg, im_depth_data.*im_mask_neg);
figure;
imshow(mod(Argb, 2^6)*(2^(16-6)-1));

% For each image in the dataset, plot it and hold it for a while so I can
% manually remove the bad ones.
disp('Plotting the rest of the samples...');
figure;
for m = 1:n_samples
    fprintf('%g of %g: %s\n', m, n_samples, DB.filenames{m});
    plotHandDepthData(DB.image_data(:,m), DB.label_data(:,m), im_width, im_height);
    pause(0.02);
end