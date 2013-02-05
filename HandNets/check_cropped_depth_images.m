clearvars; clc; close all;

dir = '../hand_depth_data_processed/';
images = ls([dir, 'hands*']);

TRAINING_IM_SIZE = 192;
NUM_COEFF = 25;
num_images = length(images(:,1));
coeffs = zeros(num_images, NUM_COEFF);
im_data = zeros(num_images, TRAINING_IM_SIZE, TRAINING_IM_SIZE);

% Collect the image data first
for i = 1:num_images
  disp(['loading image ', num2str(i), ' of ', num2str(num_images)]);
  im_data(i, :, :) = loadImageFile([dir, images(i,:)], ...
    TRAINING_IM_SIZE, TRAINING_IM_SIZE, 1, 'single');
  coeffs(i, :) = loadImageFile([dir, 'coeffr_', images(i,:)], ...
    NUM_COEFF, 1, 1, 'single');
end

% Now plot them
disp('Rendering frames at 30fps');
figure;
min_val = min(min(min(im_data)));
max_val = max(max(max(im_data)));
tic;
frame_rate = 30;
frame_time = 1 / frame_rate;
for i = 1:num_images
  % Rescale between 0 and 1
  imshow(squeeze((im_data(i, :, :) - min_val)/(max_val - min_val)));
  drawnow();
  time = toc;
  if (time < frame_time)
    pause(frame_time - time);
  end
  % Frame end, start a new frame
  tic;
end

figure;
set(gcf, 'Position', [100 100 1920 1200]);
v_size = floor(sqrt(NUM_COEFF));
u_size = ceil(NUM_COEFF / v_size);
for i = 1:NUM_COEFF
  subplot(v_size, u_size, i);
  plot(1:num_images, coeffs(:, i));
  minc = min(coeffs(:, i));
  maxc = max(coeffs(:, i));
  range = maxc - minc;
  axis([1 num_images (minc - 0.1 * range) (maxc + 0.1 * range)]);
  title(modifiedcoeff2str(i));
end