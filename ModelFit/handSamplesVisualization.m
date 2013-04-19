clearvars; close all; clc;
colormap('default')

% Indices are:
%     HAND_POS_X        = 1, 
%     HAND_POS_Y        = 2,
%     HAND_POS_Z        = 3,
%     HAND_ORIENT_X     = 4,
%     HAND_ORIENT_Y     = 5,
%     HAND_ORIENT_Z     = 6,
%     HAND_ORIENT_W     = 7,
%     WRIST_THETA       = 8,
%     WRIST_PHI         = 9,
%     THUMB_THETA       = 10,
%     THUMB_PHI         = 11,
%     THUMB_K1_THETA    = 12,
%     THUMB_K1_PHI      = 13,
%     THUMB_K2_PHI      = 14,
%     F0_THETA          = 15,
%     F0_PHI            = 16,
%     F0_K1_PHI         = 17,
%     F0_K2_PHI         = 18,
%     F1_THETA          = 19,
%     F1_PHI            = 20,
%     F1_K1_PHI         = 21,
%     F1_K2_PHI         = 22,
%     F2_THETA          = 23,
%     F2_PHI            = 24,
%     F2_K1_PHI         = 25,
%     F2_K2_PHI         = 26,
%     F3_THETA          = 27,
%     F3_PHI            = 28,
%     F3_K1_PHI         = 29,
%     F3_K2_PHI         = 30,

plot_index1 = 15;
plot_index2 = 16;

num_coeff = 30;
raw_data = csvread('hand_depth_samples/sample_point1.csv');
center_point = raw_data(1,:);

ignore_indices = zeros(num_coeff-2:1);
cur_index = 1;
for i=1:num_coeff; 
  if i~=plot_index1 && i~=plot_index2; 
    ignore_indices(cur_index) = i; 
    cur_index = cur_index + 1; 
  end
end

num_data_point = 1;
for i = 2:length(raw_data(:,1))
  keep_point = 1;
  for j = 1:length(ignore_indices)
    if (raw_data(i,ignore_indices(j)) ~= center_point(ignore_indices(j)))
      keep_point = 0;
      break;
    end
  end
  if (keep_point == 1)
    plot_x(num_data_point) = raw_data(i, plot_index1);
    plot_y(num_data_point) = raw_data(i, plot_index2);
    plot_z(num_data_point) = raw_data(i, end);
    num_data_point = num_data_point + 1;
  end
end

% Next collect the unique values of x
x = unique(plot_x);
y = unique(plot_y);
z = reshape(plot_z, length(y), length(x));
surf(x, y, z); hold on;
xlabel(['Coeff ', num2str(plot_index1)]);
ylabel(['Coeff ', num2str(plot_index2)]);
zlabel('Residual');

% Plot a sphere at the center
% [x_sphere, y_sphere, z_sphere] = sphere(10);
% x_sphere = x_sphere * 0.04 * (max(x) - min(x)) + center_point(plot_index1);
% y_sphere = y_sphere * 0.04 * (max(y) - min(y)) + center_point(plot_index2);
% z_sphere = z_sphere * 0.04 * (max(max(z)) - min(min(z))) + center_point(end);
% surf(x_sphere,y_sphere,z_sphere);
plot3(center_point(plot_index1), center_point(plot_index2), center_point(end), 'wo', 'LineWidth', 2);