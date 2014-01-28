clearvars; close all; clc;
format long;

tile_x = 8;
tile_y = 8;
ntiles = tile_x * tile_y;
dim = 512;
max_depth = 30;

kinect_data = loadImageFile('kinect_texture_tiled.bin', dim * tile_x, dim * tile_y, 1, 'float');
% figure; imshow(kinect_data); title('Kinect Depth');

rendered_depth = loadImageFile('synth_texture_tiled.bin', dim * tile_x, dim * tile_y, 1, 'float');
% figure; imshow(rendered_depth); title('Rendered Depth');

cur_image = 1;
max_depth_mat = ones(dim * tile_y, dim * tile_x, 'single') * max_depth;
residue = min(abs(kinect_data - rendered_depth), max_depth_mat);
% figure; imshow(residue); title('Ground Truth Residue');

%% First residue texture
residue_texture = loadImageFile('residue_texture_tiled.bin', dim * tile_x, dim * tile_y, 1, 'float');
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
data_term = zeros(tile_x, tile_y);
for u = 0:(tile_x-1)
    for v = 0:(tile_y-1)
        u_range = u * dim + 1 : (u + 1) * dim;
        v_range = v * dim + 1 : (v + 1) * dim;
        depth_integral = sum(sum(residue(u_range, v_range)));
        data_term(u+1,v+1) = DATA_TERM_LAMBDA * (depth_integral / (UNION + EPSILON));
    end
end

data_term
