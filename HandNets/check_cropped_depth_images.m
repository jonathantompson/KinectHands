clearvars; clc; close all;

% dir = '../data/hand_depth_data_processed/';
dir = '../data/';
images = ls([dir, 'bighands*']);

HAND_IM_SIZE_W = 640;
HAND_IM_SIZE_H = 480;
DOWNSAMPLE_FACTOR = 1;

TRAINING_IM_SIZE_W = HAND_IM_SIZE_W / DOWNSAMPLE_FACTOR;
TRAINING_IM_SIZE_H = HAND_IM_SIZE_H / DOWNSAMPLE_FACTOR;
NUM_COEFF = 25;
num_images = length(images(:,1));
coeffs = zeros(num_images, NUM_COEFF);
im_data = zeros(num_images, TRAINING_IM_SIZE_H, TRAINING_IM_SIZE_W);

% Collect the image data first
for i = 1:num_images
  cur_file = [dir, images(i,:)];
  disp(['loading image ', num2str(i), ' of ', num2str(num_images), ...
    '  <-- ', cur_file]);
  im_data(i, :, :) = loadImageFile(cur_file, ...
    TRAINING_IM_SIZE_W, TRAINING_IM_SIZE_H, 1, 'single');
  if (~isempty(ls([dir, 'coeffr_', images(i,:)])))
    coeffs(i, :) = loadImageFile([dir, 'coeffr_', images(i,:)], ...
      NUM_COEFF, 1, 1, 'single');
  end
end

% Now plot them
disp('Rendering frames at 30fps');
figure;
wnd_size = 600;
mag = (wnd_size / TRAINING_IM_SIZE_H) * 100;
set(gcf, 'Position', [200 200 wnd_size wnd_size]);
min_val = min(min(min(im_data)));
max_val = max(max(max(im_data)));
tic;
frame_rate = 30;
frame_time = 1 / frame_rate;
norm_im_data = zeros(num_images, TRAINING_IM_SIZE_H, TRAINING_IM_SIZE_W);
for i = 1:num_images
  % Rescale between 0 and 1
  norm_im_data(i, :, :) = (im_data(i, :, :) - min_val)/(max_val - min_val);
  imshow(squeeze(norm_im_data(i, :, :)),...
    'InitialMagnification', mag);
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
  range = maxc - minc + 0.000001;
  axis([1 (num_images+1) (minc - 0.1 * range) (maxc + 0.1 * range)]);
  title(modifiedcoeff2str(i));
end

% images(1,:)
% squeeze(im_data(1,40:43,40:43))
% 
% figure;
% imshow(squeeze(norm_im_data(1,:,:)),'InitialMagnification', mag);
% 
% figure;
% imshow(squeeze(norm_im_data(2,:,:)),'InitialMagnification', mag);
% 
% figure;
% diff = norm_im_data(1,:,:) - norm_im_data(2,:,:);
% imshow(squeeze(diff),'InitialMagnification', mag);
% 
% c1 = [-233.8011322021, -206.1944122314, 892.3350830078, 0.5744072199, -0.8048754334, 0.0646317750, 0.1343680471, -0.1822888851, -0.0027480125, 0.3499996662, 0.2252395153, 0.1803681850, 0.2675778866, -0.2990078926, -0.3986699581, 0.3192386627, 0.3192811012, -0.1466495991, 0.4699969292, 0.2644617558, -0.0523409843, 0.3852984905, 0.2531661987, 0.0456080437, -0.0189075470, 0.2831306458];
% c2 = [-233.8011322021, -206.1944122314, 892.3350830078, 0.5744072199, -0.8048754334, 0.0646317750, 0.1343680471, -0.1822888851, -0.0027480125, 0.3499996662, 0.2252395153, 0.1803681850, 0.2675778866, -0.2990078926, -0.3986699581, 0.3192386627, 0.3192811012, -0.1466495991, 0.4699969292, 0.2644617558, -0.0523409843, 0.3852984905, 0.2531661987, 0.0456080437, -0.0189075470, 0.2831306458];

