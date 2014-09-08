% Jonathan Tompson, New York University - 8/28/2014
% This is a simple example script to visualize an example
clearvars; close all; clc; rng(0);

dataset_dir = 'dataset\test\';
load([dataset_dir, 'joint_data.mat']);
num_kinects = size(joint_uvd, 1);
num_images = size(joint_uvd, 2);

conv_joint_names = {'PALM_3', 'PALM_1', 'PALM_2', 'TH_KNU3_A', 'TH_KNU3_B', ...
  'TH_KNU2_B', 'F1_KNU3_A', 'F1_KNU2_B', 'F2_KNU3_A', 'F2_KNU2_B', ...
  'F3_KNU3_A', 'F3_KNU2_B', 'F4_KNU3_A', 'F4_KNU2_B'};

% Create a mapping from the convnet joint indices to the dataset indices
conv_to_dataset = zeros(1, length(conv_joint_names));
for i = 1:length(conv_joint_names)
  for j = 1:length(joint_names)
    if strcmp(joint_names{j}, conv_joint_names{i})
      conv_to_dataset(i) = j;
    end
  end
end

% Get the probability + uv data
fileID = fopen('puv_im_space.bin','r');
conv_joint_uvconf = fread(fileID, num_images * length(conv_joint_names) * 3, 'single');
conv_joint_uvconf = reshape(conv_joint_uvconf, 3, length(conv_joint_names), num_images);
conv_joint_uvconf = permute(conv_joint_uvconf, [3 2 1]);
fclose(fileID);

% Now put them in the same order as the database joints
pred_joint_uvconf = zeros(1, num_images, length(joint_names), 3);
for j = 1:length(conv_joint_names)
  ojnt = conv_to_dataset(j);
  pred_joint_uvconf(1,:,ojnt,1:2) = conv_joint_uvconf(:,j,2:3);
  pred_joint_uvconf(1,:,ojnt,3) = conv_joint_uvconf(:,j,1);
end

save('test_predictions.mat', 'pred_joint_uvconf', 'conv_joint_names');