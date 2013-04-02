% clearvars; clc; close all;
% 
% dir = '../data/hand_depth_data_processed/';
% images = ls([dir, 'hands*']);
% 
% HAND_IM_SIZE_W = single(192);
% HAND_IM_SIZE_H = single(192);
% DOWNSAMPLE_FACTOR = single(2);
% TRAINING_IM_SIZE_W = single(HAND_IM_SIZE_W / DOWNSAMPLE_FACTOR);
% TRAINING_IM_SIZE_H = single(HAND_IM_SIZE_H / DOWNSAMPLE_FACTOR);
% NUM_COEFF = single(42);
% NUM_HPF_BANKS = single(3);
% wnd_size = single(600);
% mag = (wnd_size / TRAINING_IM_SIZE_H) * 100;
% 
% num_images = single(length(images(:,1)));
% coeffs = zeros(num_images, NUM_COEFF, 'single');
% im_data = cell(1,NUM_HPF_BANKS);
% hpf_im_data = cell(1,NUM_HPF_BANKS);
% w = TRAINING_IM_SIZE_W;
% h = TRAINING_IM_SIZE_H;
% size_hfp_images = 0;
% for i = 1:NUM_HPF_BANKS
%   im_data{i} = zeros(num_images, h, w, 'single');
%   hpf_im_data{i} = zeros(num_images, h, w, 'single');
%   size_hfp_images = size_hfp_images + w * h;
%   h = h / 2;
%   w = w / 2;
% end
% 
% % Collect the image data first
% for i = 1:num_images
%   cur_file = [dir, images(i,:)];
%   cur_hpf_file = [dir, 'hpf_', images(i,:)];
%   disp(['loading image ', num2str(i), ' of ', num2str(num_images), ...
%     '  <-- ', cur_file]);
%   if (~isempty(ls([dir, 'coeffr_', images(i,:)])))
%     coeffs(i, :) = single(loadImageFile([dir, 'coeffr_', images(i,:)], ...
%       NUM_COEFF, 1, 1, 'single'));
%   else
%     disp(['WARNING: Coeff file ',dir, 'coeffr_', images(i,:),'does not exist']);
%   end
%   % Now load in the HPF image banks
%   cur_hpf_im = single(loadImageFile(cur_hpf_file, size_hfp_images, 1, 1, 'single'));
%   cur_im_data = single(loadImageFile(cur_file, size_hfp_images, 1, 1, 'single'));
%   % Save it to our local storage
%   w = TRAINING_IM_SIZE_W;
%   h = TRAINING_IM_SIZE_H;
%   cur_ind = 1;
%   for j = 1:NUM_HPF_BANKS
%     im_data{j}(i,:,:) = reshape(cur_im_data(cur_ind:cur_ind+w*h-1), w, h)';
%     hpf_im_data{j}(i,:,:) = reshape(cur_hpf_im(cur_ind:cur_ind+w*h-1), w, h)';
%     cur_ind = cur_ind + w * h;
%     h = h / 2;
%     w = w / 2;
%   end
% end
% 
% for i = 1:num_images
%   % Check that the image is not empty
%   stdev = sqrt(var(var(im_data{1}(i, :, :))));
%   if ((stdev < 0.001) || (sum(sum(isnan(im_data{1}(i, :, :))) ~= 0)))
%     disp('WARNING: Potential bad image:');
%     disp(['  stdev = ', num2str(stdev)]);
%     disp(['  index = ', num2str(i)]);
%     disp(['  filename = ', [dir, images(i,:)]]);
%     disp('Deleting it!');
%     delete([dir, images(i,:)]);
%     delete([dir, 'coeffr_', images(i,:)]);
%     delete([dir, 'coeffl_', images(i,:)]);
%   end
% end
%   
% % Now plot them
% disp('Rendering frames at 60fps');
% figure;
% set(gcf, 'Position', [200 200 1.5*(wnd_size+200) (wnd_size+200)]);
% tic;
% frame_rate = single(60);
% frame_time = single(1 / frame_rate);
% for i = 1:num_images
%   clf;
%   if mod(i,100) == 0
%     disp(['Image ',num2str(i), ' of ', num2str(num_images)]);
%   end
%   subplot(2,3,1);
%   imshow(squeeze(im_data{1}(i, :, :)), 'InitialMagnification', mag);
%   subplot(2,3,2);
%   imshow(squeeze(im_data{2}(i, :, :)), 'InitialMagnification', mag);
%   subplot(2,3,3);
%   imshow(squeeze(im_data{3}(i, :, :)), 'InitialMagnification', mag);
%   
%   subplot(2,3,4);
%   imshow(squeeze(hpf_im_data{1}(i, :, :)) + 0.5, 'InitialMagnification', mag);
%   subplot(2,3,5);
%   imshow(squeeze(hpf_im_data{2}(i, :, :)) + 0.5, 'InitialMagnification', mag);
%   subplot(2,3,6);
%   imshow(squeeze(hpf_im_data{3}(i, :, :)) + 0.5, 'InitialMagnification', mag);
%   drawnow();
%   time = single(toc);
%   if (time < frame_time)
%     pause(frame_time - time);
%   end
%   % Frame end, start a new frame
%   tic;
% end
% 
% figure;
% tic;
% frame_rate = single(60);
% frame_time = single(1 / frame_rate);
% % for i = 1:num_images
% for i = 8900:num_images 
%   if mod(i,100) == 0
%     disp(['Image ',num2str(i), ' of ', num2str(num_images)]);
%   end
%   cur_image = squeeze(im_data{1}(i, :, :));
%   clf;
%   imshow(cur_image, 'InitialMagnification', mag);
%   hold on;
%   % Draw the crosses
%   draw_cross(coeffs(i,1:2), 2, size(cur_image));
%   for j = 13:2:NUM_COEFF
%     draw_cross(coeffs(i,j:j+1), 2, size(cur_image));
%   end
%   drawnow();
%   time = single(toc);
%   if (time < frame_time)
%     pause(frame_time - time);
%   end
%   % Frame end, start a new frame
%   tic;
% end
% 
% % Plot the coefficients
% figure;
% set(gcf, 'Position', [100 100 1920 1200]);
% u_size = double(int32(floor(sqrt(NUM_COEFF))));
% v_size = double(int32(ceil(NUM_COEFF / u_size)));
% for i = double(1:NUM_COEFF)
%   subplot(v_size, u_size, i);
%   plot(1:num_images, coeffs(:, i)'); grid on;
%   minc = min(coeffs(:, i));
%   maxc = max(coeffs(:, i));
%   range = maxc - minc + 0.000001;
%   axis([1 (num_images+1) (minc - 0.1 * range) (maxc + 0.1 * range)]);
%   title(modifiedcoeff2str(i));
% end
% 
% %% Delete some stuff
% if 1 == 0
%   for i = (4400*2):(5025*2)
%     cur_file = [dir, images(i,:)];
%     delete(cur_file);
%     cur_file = [dir, 'coeffr_', images(i,:)];
%     delete(cur_file);
%     cur_file = [dir, 'coeffl_', images(i,:)];
%     delete(cur_file);
%   end
% end
%   
% %% Try out our HPF
% close all; clc;
% test_image = single(40);
% im = squeeze(im_data{1}(test_image,:,:));
% figure;
% imshow(im, 'InitialMagnification', mag);
% figure;
% surf(double(im));
% shading interp;
% 
% sigma_pix = single(1.5);
% radius = ceil(single(3 * sigma_pix));  % choose kernel radius to be 2 sigma
% size = single(max(7, 2*radius + 1));
% center = single(size/2 + 0.5);
% gauss = single(exp(-((((1:size) - center) / sigma_pix).^2)/2));
% gauss_cont = single(exp(-((((1:0.0001:size) - center) / sigma_pix).^2)/2));
% % figure;
% % plot(1:size, gauss); hold on; plot(1:0.0001:size, gauss_cont, 'r');
% 
% % Create scaling coeff by convolving an image of all ones, and clipping
% % the end points
% ones_im = ones(TRAINING_IM_SIZE_H, TRAINING_IM_SIZE_W, 'single');
% coeffs = filter_zero_bndry(ones_im, gauss);
% ones_im =  ones(TRAINING_IM_SIZE_H/2, TRAINING_IM_SIZE_W/2, 'single');
% downs2_coeffs = filter_zero_bndry(ones_im, gauss);
% ones_im = ones(TRAINING_IM_SIZE_H/4, TRAINING_IM_SIZE_W/4, 'single');
% downs4_coeffs = filter_zero_bndry(ones_im, gauss);
% 
% %% Now filter the image
% filt_im = filter_zero_bndry(im, gauss);
% filt_im = filt_im ./ coeffs;
% % figure;
% % imshow(filt_im, 'InitialMagnification', mag);
% 
% % Now create the high pass by sub off the gaussian image
% hp_filt_im = (im - filt_im) * 2;
% figure;
% imshow(hp_filt_im+0.5, 'InitialMagnification', mag);
% figure;
% surf(double(hp_filt_im+0.5));
% shading interp;
% 
% % Check that the image is the same as the C++ version
% diff = hp_filt_im - squeeze(hpf_im_data{1}(test_image,:,:));
% if (max(max(diff)) > 1e-6)
%   disp('C++ and matlab HFP images dont match!');
%   figure;
%   imshow((diff-min(min(diff)))/(max(max(diff))-min(min(diff))), 'InitialMagnification', mag)
% else
%   disp(['C++ and matlab max difference is: ', num2str(max(max(diff)))]);
% end
% 
% %% Downsample by 2
% downs2_im = downsample(im, 2);
% % figure;
% % imshow(downs2_im, 'InitialMagnification', 2 * mag);
% 
% % Now filter the image
% downs2_filt_im = filter_zero_bndry(downs2_im, gauss);
% downs2_filt_im = downs2_filt_im ./ downs2_coeffs;
% % figure;
% % imshow(downs2_filt_im, 'InitialMagnification', 2 * mag);
% 
% % Now create the high pass by sub off the gaussian image
% hp_downs2_filt_im = (downs2_im - downs2_filt_im) * 2;
% figure;
% imshow(hp_downs2_filt_im+0.5, 'InitialMagnification', 2 * mag);
% figure;
% surf(double(hp_downs2_filt_im+0.5));
% shading interp;
% 
% % Check that the image is the same as the C++ version
% diff = hp_downs2_filt_im - squeeze(hpf_im_data{2}(test_image,:,:));
% if (max(max(diff)) > 1e-6)
%   figure;
%   imshow((diff-min(min(diff)))/(max(max(diff))-min(min(diff))), 'InitialMagnification', mag * 2)
%   disp('C++ and matlab HFP images dont match!');
% else
%   disp(['C++ and matlab max difference is: ', num2str(max(max(diff)))]);
% end
% 
% %% Downsample again by 2
% downs4_im = downsample(downs2_im, 2);
% % figure;
% % imshow(downs4_im, 'InitialMagnification', 4 * mag);
% 
% % Now filter the image
% downs4_filt_im = filter_zero_bndry(downs4_im, gauss);
% downs4_filt_im = downs4_filt_im ./ downs4_coeffs;
% % figure;
% % imshow(downs4_filt_im, 'InitialMagnification', 4 * mag);
% 
% % Now create the high pass by sub off the gaussian image
% hp_downs4_filt_im = (downs4_im - downs4_filt_im) * 2;
% figure;
% imshow(hp_downs4_filt_im+0.5, 'InitialMagnification', 4 * mag);
% figure;
% surf(double(hp_downs4_filt_im+0.5));
% shading interp;
% 
% % Check that the image is the same as the C++ version
% diff = hp_downs4_filt_im - squeeze(hpf_im_data{3}(test_image,:,:));
% if (max(max(diff)) > 1e-6)
%   figure;
%   imshow((diff-min(min(diff)))/(max(max(diff))-min(min(diff))), 'InitialMagnification', mag * 4)
%   disp('C++ and matlab HFP images dont match!');
% else
%   disp(['C++ and matlab max difference is: ', num2str(max(max(diff)))]);
% end

%% Check the other images:
clearvars; clc; close all;
cur_file = 'unscaled_depth_im.bin';
depth_image = loadImageFile(cur_file, 256, 256, 1, 'single');
imshow(depth_image, 'InitialMagnification', 200)
figure;
imshow(depth_image==1, 'InitialMagnification', 200)
cur_file = 'scaled_depth_im.bin';
depth_image_scaled = loadImageFile(cur_file, 96, 96, 1, 'single');
figure;
imshow(depth_image_scaled, 'InitialMagnification', 2.6667*200)
figure;
imshow(depth_image_scaled==1, 'InitialMagnification', 2.6667*200)

