clearvars; clc; close all;

time_step = 1/30;
t = 0:time_step:40;
perlin_noise_time_scale = 0.1;
n_pts = length(t);
noise = zeros(3, n_pts);
for i = 1:3
  noise(i,:) = perlin_noise_2d(((i - 1) * 100 + rand(1,1)) * ones(1, n_pts), t);
end
plot(t, noise);

fov_deg = single(60);
width = single(640);
height = single(480);
znear = single(-1);
zfar = single(-2000);

base_hand_pos = single([0, 0, -200]);
hand_size = single(20);

figure;
p_mat = glProjection(znear, zfar, fov_deg, width, height);
quad_vert = single([[-1; -1; 0; 1], [-1; +1; 0; 1], [+1; +1; 0; 1], ...
  [+1; -1; 0; 1]]);  %% Clockwise rotation
movement_radius = [150, 150, 200];

% tic;
for i = 1:n_pts
  disp(['rendering pt ', num2str(i), ' of ', num2str(n_pts)]);
  hand_pos = base_hand_pos + noise(:,i)' .* movement_radius;
  hand_pos = floor(hand_pos);
  cur_im = create_toy_image(p_mat, quad_vert, hand_pos, hand_size, width, ...
    height);
  z_min = min(min(cur_im));
  z_max = max(max(cur_im));
  cur_im_disp = single((cur_im - z_min) / (z_max - z_min));
  imshow(cur_im_disp);
  drawnow();
%   t = toc;
%   pause(time_step - t);
%   tic;
  
  cur_im_vec = cur_im'; cur_im_vec = cur_im_vec(:);
  coeffs = zeros(1,28);
  coeffs(1:3) = hand_pos;
  file = fopen(['./toy_hand_data/coeffr_hands_', num2str(i,'%07d'), '.bin'], 'w');
  fwrite(file, coeffs, 'single');
  fclose(file);
  file = fopen(['./toy_hand_data/depth_only_hands_', num2str(i,'%07d'), '.bin'], 'w');
  fwrite(file, cur_im_vec, 'int16');
  fclose(file);
end