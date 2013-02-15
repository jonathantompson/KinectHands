clearvars; clc; close all;

dir = '../data/hand_depth_data_processed/';
images = ls([dir, 'hands*']);

HAND_IM_SIZE_W = 192;
HAND_IM_SIZE_H = 192;
DOWNSAMPLE_FACTOR = 2;

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

for i = 1:num_images
  % Check that the image is not empty
  stdev = sqrt(var(var(im_data(i, :, :))));
  if ((stdev < 0.001) || (sum(sum(isnan(im_data(i, :, :))) ~= 0)))
    disp('WARNING: Potential bad image:');
    disp(['  stdev = ', num2str(stdev)]);
    disp(['  index = ', num2str(i)]);
    disp(['  filename = ', [dir, images(i,:)]]);
    disp('Deleting it!');
    delete([dir, images(i,:)]);
    delete([dir, 'coeffr_', images(i,:)]);
    delete([dir, 'coeffl_', images(i,:)]);
  end
end
  
% Now plot them
disp('Rendering frames at 60fps');
figure;
wnd_size = 600;
mag = (wnd_size / TRAINING_IM_SIZE_H) * 100;
set(gcf, 'Position', [200 200 wnd_size wnd_size]);
min_val = min(min(min(im_data)));
max_val = max(max(max(im_data)));
tic;
frame_rate = 60;
frame_time = 1 / frame_rate;
norm_im_data = (im_data - min_val)/(max_val - min_val);
for i = 1:num_images
  if mod(i,100) == 0
    disp(['Image ',num2str(i), ' of ', num2str(num_images)]);
  end
  % Rescale between 0 and 1
  imshow(squeeze(norm_im_data(i, :, :)), 'InitialMagnification', mag);
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

%% Delete some stuff
if 1 == 0
  for i = (4400*2):(5025*2)
    cur_file = [dir, images(i,:)];
    delete(cur_file);
    cur_file = [dir, 'coeffr_', images(i,:)];
    delete(cur_file);
    cur_file = [dir, 'coeffl_', images(i,:)];
    delete(cur_file);
  end
end
  