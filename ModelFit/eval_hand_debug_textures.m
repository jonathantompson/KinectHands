clearvars; close all; clc;
format long;
EPSILON = 0.0000001;
lambda = 0.0025;

tile_x = 1;
tile_y = 1;
ntiles = tile_x * tile_y;
im_width = 512;
im_height = 424;
residue_dim = max(im_width, im_height);
max_depth = 30;

kinect_data = single(loadImageFile('kinect_texture.bin', im_width * tile_x, im_height * tile_y, 1, 'int16'));
figure; imshow(kinect_data); title('Kinect Depth');

rendered_depth = loadImageFile('synth_texture.bin', im_width * tile_x, im_height * tile_y, 1, 'float');
figure; imshow(rendered_depth); title('Rendered Depth');

cur_image = 1;
max_depth_mat = ones(im_height * tile_y, im_width * tile_x, 'single') * max_depth;
residue = zeros(residue_dim * tile_y, residue_dim * tile_x, 'single');
residue(1:im_height,1:im_width) = min(abs(kinect_data - rendered_depth), max_depth_mat);
figure; imshow(residue); title('Ground Truth Residue');

%% First residue texture
residue_texture = loadImageFile('residue_texture.bin', residue_dim * tile_x, residue_dim * tile_y, 1, 'float');
figure; imshow(residue_texture); title('Model Fit Residue');

delta_res = residue_texture - residue;
figure; imshow(delta_res / max(max(delta_res))); title('Delta Residue');

sum(sum(delta_res))