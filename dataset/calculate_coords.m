% Jonathan Tompson, New York University - 8/28/2014
% This is a simple example script to visualize an example
clearvars; close all; clc; rng(0);

dataset_dir = '../data/dataset/';
num_kinects = length(cellstr(ls([dataset_dir 'depth_*_0000001_.png'])));

% Get the image names
for k = 1:num_kinects
  if k == 1
    uvd_fn{k} = cellstr(ls([dataset_dir 'uvd_',num2str(k),'_*_.bin']));
  else
    uvd_fn{k} = cellstr(ls([dataset_dir 'mat1matk_',num2str(k),'_*_.bin']));
    assert(length(uvd_fn{k}) == length(uvd_fn{1}));
  end
end
num_images = length(uvd_fn{1});

joint_names = {'F1_KNU3_A','F1_KNU3_B','F1_KNU2_A','F1_KNU2_B','F1_KNU1_A','F1_KNU1_B',...
  'F2_KNU3_A','F2_KNU3_B','F2_KNU2_A','F2_KNU2_B','F2_KNU1_A','F2_KNU1_B',...
  'F3_KNU3_A','F3_KNU3_B','F3_KNU2_A','F3_KNU2_B','F3_KNU1_A','F3_KNU1_B',...
  'F4_KNU3_A','F4_KNU3_B','F4_KNU2_A','F4_KNU2_B','F4_KNU1_A','F4_KNU1_B',...
  'TH_KNU3_A','TH_KNU3_B','TH_KNU2_A','TH_KNU2_B','TH_KNU1_A','TH_KNU1_B',...
  'PALM_1','PALM_2','PALM_3','PALM_4','PALM_5','PALM_6'};
joint_uvd = zeros(num_kinects, num_images, length(joint_names), 3);
joint_xyz = zeros(num_kinects, num_images, length(joint_names), 3);
for i = 1:num_images
  % Load in the UVD locations
  disp(uvd_fn{1}{i});
  fid = fopen([dataset_dir, uvd_fn{1}{i}], 'r'); 
  [uvd, count] = fread(fid, 'float');
  fclose(fid);
  assert(count == length(joint_names) * 3);  % Just in case
  uvd = reshape(uvd, 3, length(joint_names))';
  xyz = squeeze(convert_uvd_to_xyz(reshape(uvd, 1, length(joint_names), 3)));
  
  joint_uvd(1, i, :, :) = uvd;
  joint_xyz(1, i, :, :) = xyz;
  
%   % Visualize it
%   jnt_colors = rand(length(joint_names), 3);
%   scatter3(xyz(:,1), xyz(:,3), xyz(:,2), 10, jnt_colors, 'Fill','LineWidth', 0.5);
  
  % Now transform coordinates into the other camera coordinate systems
  xyz(:,4) = 1;  % homogeneous coords
  for k = 2:num_kinects
    % Open the file containing the camera 1 and camera k matrices
    disp(uvd_fn{k}{i});
    fid = fopen([dataset_dir, uvd_fn{k}{i}], 'r'); 
    [mats, count] = fread(fid, 'float');
    fclose(fid);
    assert(count == 4 * 4 * 2);  % Just in case
    mat1 = reshape(mats(1:16),4,4);
    matk = reshape(mats(17:32),4,4);
    xyzk = (inv(matk) * mat1 * xyz')';
    xyzk = xyzk(:,1:3);  % Non homogeneous coords
    uvdk = squeeze(convert_xyz_to_uvd(reshape(xyzk, 1, length(joint_names), 3)));
    
    joint_uvd(k,i,:,:) = uvdk;
    joint_xyz(k,i,:,:) = xyzk;
  end
end

save([dataset_dir, 'joint_data.mat'], 'joint_names', 'joint_uvd', 'joint_xyz');

