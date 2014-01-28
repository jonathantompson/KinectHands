clearvars; close all; clc;
format long;

dim = 512;
max_depth = 30;

kinect_data = loadImageFile('kinect_texture.bin', dim, dim, 1, 'float');

rendered_depth = loadImageFile('synth_texture.bin', dim, dim, 1, 'float');
zmin = min(min(rendered_depth(find(rendered_depth~=0))));
zmax = max(max(rendered_depth));
figure; imshow((rendered_depth - zmin) / (zmax-zmin)); title('Rendered Depth');
figure; imshow((kinect_data - zmin) / (zmax-zmin)); title('Kinect Depth');

cur_image = 1;
max_depth_mat = ones(dim, dim, 'single') * max_depth;
residue = min(abs(kinect_data - rendered_depth), max_depth_mat);
% figure; imshow(residue); title('Ground Truth Residue');

%% First residue texture
residue_texture = loadImageFile('residue_texture.bin', dim, dim, 1, 'float');
% figure; imshow(residue_texture); title('Model Fit Residue');

delta_res = residue_texture - residue;
% figure; imshow(delta_res / max(max(delta_res))); title('Delta Residue');

display('residue error:');
sum(sum(delta_res))

display('ground truth objective function value:');
display(' (within floating point roundoff)');
display('ALSO ASSUMES NO CONSTRAINTS ARE ACTIVE!!!');
UNION = 1000000.0;
DATA_TERM_LAMBDA = 0.2;
FLT_EPSILON = 1.192092896e-07;
EPSILON = 2 * FLT_EPSILON;
depth_integral = sum(sum(residue));
data_term = DATA_TERM_LAMBDA * (depth_integral / (UNION + EPSILON))
