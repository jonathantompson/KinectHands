clearvars; close all; clc;
format long;
EPSILON = 0.0000001;
lambda = 0.0025;

tile_x = 8;
tile_y = 8;
ntiles = tile_x * tile_y;
im_width = 640;
im_height = 480;
max_depth = 80;

kinect_data = loadImageFile('kinect_depth_texture_tiled.bin', im_width * tile_x, im_height * tile_y, 1, 'float');
% figure; imshow(kinect_data); title('Kinect Depth');
depth = kinect_data(1:480, 1:640);
d_k_max = max(max(depth));
d_k_min = min(min(depth(depth ~= 0)));
depth_scaled = (depth-d_k_min)/(d_k_max-d_k_min);
depth_scaled(depth == 0) = 1;
imshow(depth_scaled); title('Kinect Depth');

rendered_depth = loadImageFile('depth_texture_tiled.bin', im_width * tile_x, im_height * tile_y, 1, 'float');
% figure; imshow(rendered_depth); title('Rendered Depth');
% depth = rendered_depth;
depth = rendered_depth;
d_k_max = max(max(depth));
d_k_min = min(min(depth(depth ~= 0)));
depth_scaled = (depth-d_k_min)/(d_k_max-d_k_min);
depth_scaled(depth == 0) = 1;
imshow(depth_scaled); title('Rendered Depth');

cur_image = 1;
delta_d = zeros(tile_x, tile_y, 'single');
union = zeros(tile_x, tile_y, 'single');
intersection = zeros(tile_x, tile_y, 'single');
max_depth_mat = ones(im_height * tile_y, im_width * tile_x, 'single') * max_depth;
delta = min(abs(kinect_data - rendered_depth), max_depth_mat);
delta = delta .* delta;
data_sum = abs(kinect_data) + abs(rendered_depth);
data_mult = (abs(kinect_data) > EPSILON) .* (abs(rendered_depth) > EPSILON);
for cur_tile_y = 1:tile_y
    y_offset = im_height * (cur_tile_y - 1) + 1;
    for cur_tile_x = 1:tile_x
        x_offset = im_width * (cur_tile_x - 1) + 1;
        disp(['Calculating correct values for im ',num2str(cur_image),' of ',num2str(ntiles)]);
        delta_d(cur_tile_y, cur_tile_x) = sum(sum(delta(y_offset:(y_offset+im_height-1), x_offset:(x_offset+im_width-1))));
        union(cur_tile_y, cur_tile_x) = sum(sum(data_sum(y_offset:(y_offset+im_height-1), x_offset:(x_offset+im_width-1)) > EPSILON));
        intersection(cur_tile_y, cur_tile_x) = sum(sum(data_mult(y_offset:(y_offset+im_height-1), x_offset:(x_offset+im_width-1))));
        cur_image = cur_image + 1;
    end
end
intersection_correct = intersection;
union_correct = union;
delta_d_correct = delta_d;

%% First residue texture
[residue_texture_1, delta_d_1, union_1, intersect_1] = loadImageFile('residue_texture_x8_tiled.bin', im_width * tile_x, im_height * tile_y, 3, 'float');
delta_d_1 = single(delta_d_1);
% figure; imshow(residue_texture_1); title('residue texture 1');
depth = residue_texture_1(1:2*480,1:2*640, :);
d = squeeze(depth(:,:,1));
d_k_max = max(max(d));
d_k_min = min(min(d(d ~= 0)));
depth_scaled = depth;
depth_scaled(:,:,1) = (depth(:,:,1) - d_k_min)/(d_k_max-d_k_min);
imshow(depth_scaled); title('Rendered Depth');

delta_d_residue_tex_1 = zeros(tile_x, tile_y, 'single');
union_residue_tex_1 = zeros(tile_x, tile_y, 'single');
intersection_residue_tex_1 = zeros(tile_x, tile_y, 'single');
for cur_tile_y = 1:tile_y
    y_offset = im_height * (cur_tile_y - 1) + 1;
    for cur_tile_x = 1:tile_x
        x_offset = im_width * (cur_tile_x - 1) + 1;
        intersection_residue_tex_1(cur_tile_y, cur_tile_x) = sum(sum(intersect_1(y_offset:(y_offset+im_height-1), x_offset:(x_offset+im_width-1))));
        union_residue_tex_1(cur_tile_y, cur_tile_x) = sum(sum(union_1(y_offset:(y_offset+im_height-1), x_offset:(x_offset+im_width-1))));
        delta_d_residue_tex_1(cur_tile_y, cur_tile_x) = sum(sum(delta_d_1(y_offset:(y_offset+im_height-1), x_offset:(x_offset+im_width-1))));
    end
end

ok = 1;
if (sum(sum(delta_d_residue_tex_1 == delta_d_correct)) ~= ntiles)
    disp('Residue delta_d_1 texture is incorrect!'); ok = 0;
end
if (sum(sum(union_residue_tex_1 == union_correct)) ~= ntiles)
    disp('Residue union_residue_tex_1 texture is incorrect!'); ok = 0;
end
if (sum(sum(intersection_residue_tex_1 == intersection_correct)) ~= ntiles)
    disp('Residue intersection_residue_tex_1 texture is incorrect!'); ok = 0;
end
if (ok == 1)
    disp('residue_texture_x8.bin correct');
end

%% First Downsample
[residue_texture_2, delta_d_2, union_2, intersect_2] = loadImageFile('residue_texture_x2_tiled.bin', im_width * tile_x / 4, im_height * tile_y / 4, 3, 'float');
delta_d_2 = single(delta_d_2);
% imshow(residue_texture_2./max(max(max(residue_texture_2)))); title('residue_texture_2');

% Downsample manually here (to check if it's correct --> Floating point
% roundoff means that there might be differences between Matlab and shader)
delta_d_2_manual = zeros(im_height * tile_x / 4, im_width * tile_y / 4, 'single');
for v = 1:im_height * tile_x / 4
    for u = 1:im_width * tile_x / 4
        v_range = ((v-1)*4 + 1):((v-1)*4 + 4);
        u_range = ((u-1)*4 + 1):((u-1)*4 + 4);
        delta_d_2_manual(v, u) = sum(sum(delta_d_1(v_range, u_range)));
    end
end

delta_d_residue_tex_2_manual = zeros(tile_x, tile_y, 'single');
delta_d_residue_tex_2 = zeros(tile_x, tile_y, 'single');
union_residue_tex_2 = zeros(tile_x, tile_y, 'single');
intersection_residue_tex_2 = zeros(tile_x, tile_y, 'single');
for cur_tile_y = 1:tile_y
    y_offset = (im_height / 4) * (cur_tile_y - 1) + 1;
    for cur_tile_x = 1:tile_x
        x_offset = (im_width / 4) * (cur_tile_x - 1) + 1;
        intersection_residue_tex_2(cur_tile_y, cur_tile_x) = sum(sum(intersect_2(y_offset:(y_offset+im_height/4-1), x_offset:(x_offset+im_width/4-1))));
        union_residue_tex_2(cur_tile_y, cur_tile_x) = sum(sum(union_2(y_offset:(y_offset+im_height/4-1), x_offset:(x_offset+im_width/4-1))));
        delta_d_residue_tex_2(cur_tile_y, cur_tile_x) = sum(sum(delta_d_2(y_offset:(y_offset+im_height/4-1), x_offset:(x_offset+im_width/4-1))));
        delta_d_residue_tex_2_manual(cur_tile_y, cur_tile_x) = sum(sum(delta_d_2_manual(y_offset:(y_offset+im_height/4-1), x_offset:(x_offset+im_width/4-1))));
    end
end

ok = 1;
% I think that floats on video card aren't handled the same... for some
% reason there is a very small offset between the matlab and openGL float
if (sum(sum(abs(delta_d_residue_tex_2_manual - delta_d_residue_tex_2) ./abs(delta_d_residue_tex_2) < 1e-7)) ~= ntiles)
    disp('Residue delta_d_2 texture is incorrect!'); ok = 0;
end
if (sum(sum(union_residue_tex_2 == union_correct)) ~= ntiles)
    disp('Residue union_residue_tex_2 texture is incorrect!'); ok = 0;
end
if (sum(sum(intersection_residue_tex_2 == intersection_correct)) ~= ntiles)
    disp('Residue intersection_residue_tex_2 texture is incorrect!'); ok = 0;
end
if (ok == 1)
    disp('residue_texture_x2.bin correct');
end

%% Second Downsample
[residue_texture_3, delta_d_3, union_3, intersect_3] = loadImageFile('residue_texture_2_tiled.bin', im_width * tile_x / 16, im_height * tile_y / 16, 3, 'float');
delta_d_3 = single(delta_d_3);
% imshow(residue_texture_3./max(max(max(residue_texture_3)))); title('residue_texture_3');

delta_d_3_manual = zeros(im_height * tile_y / 16, im_width * tile_x / 16, 'single');
for v = 1:im_height * tile_x / 16
    for u = 1:im_width * tile_x / 16
        v_range = ((v-1)*16 + 1):((v-1)*16 + 16);
        u_range = ((u-1)*16 + 1):((u-1)*16 + 16);
        delta_d_3_manual(v, u) = sum(sum(delta(v_range, u_range)));
    end
end

delta_d_residue_tex_3_manual = zeros(tile_x, tile_y, 'single');
delta_d_residue_tex_3 = zeros(tile_x, tile_y, 'single');
union_residue_tex_3 = zeros(tile_x, tile_y, 'single');
intersection_residue_tex_3 = zeros(tile_x, tile_y, 'single');
for cur_tile_y = 1:tile_y
    y_offset = (im_height / 16) * (cur_tile_y - 1) + 1;
    for cur_tile_x = 1:tile_x
        x_offset = (im_width / 16) * (cur_tile_x - 1) + 1;
        intersection_residue_tex_3(cur_tile_y, cur_tile_x) = sum(sum(intersect_3(y_offset:(y_offset+im_height/16-1), x_offset:(x_offset+im_width/16-1))));
        union_residue_tex_3(cur_tile_y, cur_tile_x) = sum(sum(union_3(y_offset:(y_offset+im_height/16-1), x_offset:(x_offset+im_width/16-1))));
        delta_d_residue_tex_3(cur_tile_y, cur_tile_x) = sum(sum(delta_d_3(y_offset:(y_offset+im_height/16-1), x_offset:(x_offset+im_width/16-1))));
        delta_d_residue_tex_3_manual(cur_tile_y, cur_tile_x) = sum(sum(delta_d_3_manual(y_offset:(y_offset+im_height/16-1), x_offset:(x_offset+im_width/16-1))));
    end
end

ok = 1;
if (sum(sum(abs(delta_d_residue_tex_3_manual - delta_d_residue_tex_3) ./abs(delta_d_residue_tex_3) < 1e-6)) ~= ntiles)
    disp('Residue delta_d_3 texture is incorrect!'); ok = 0;
end
if (sum(sum(union_residue_tex_3 == union_correct)) ~= ntiles)
    disp('Residue union_residue_tex_3 texture is incorrect!'); ok = 0;
end
if (sum(sum(intersection_residue_tex_3 == intersection_correct)) ~= ntiles)
    disp('Residue intersection_residue_tex_3 texture is incorrect!'); ok = 0;
end
if (ok == 1)
    disp('residue_texture_2.bin correct');
end

%% Third Downsample
[residue_texture_4, delta_d_4, union_4, intersect_4] = loadImageFile('residue_texture_4_tiled.bin', im_width * tile_x / 32, im_height * tile_y / 32, 3, 'float');
delta_d_4 = single(delta_d_4);
% imshow(residue_texture_4./max(max(max(residue_texture_4)))); title('residue_texture_4');

delta_d_4_manual = zeros(im_height * tile_y / 32, im_width * tile_x / 32, 'single');
for v = 1:im_height * tile_x / 32
    for u = 1:im_width * tile_x / 32
        v_range = ((v-1)*32 + 1):((v-1)*32 + 32);
        u_range = ((u-1)*32 + 1):((u-1)*32 + 32);
        delta_d_4_manual(v, u) = sum(sum(delta(v_range, u_range)));
    end
end

delta_d_residue_tex_4_manual = zeros(tile_x, tile_y, 'single');
delta_d_residue_tex_4 = zeros(tile_x, tile_y, 'single');
union_residue_tex_4 = zeros(tile_x, tile_y, 'single');
intersection_residue_tex_4 = zeros(tile_x, tile_y, 'single');
for cur_tile_y = 1:tile_y
    y_offset = (im_height / 32) * (cur_tile_y - 1) + 1;
    for cur_tile_x = 1:tile_x
        x_offset = (im_width / 32) * (cur_tile_x - 1) + 1;
        intersection_residue_tex_4(cur_tile_y, cur_tile_x) = sum(sum(intersect_4(y_offset:(y_offset+im_height/32-1), x_offset:(x_offset+im_width/32-1))));
        union_residue_tex_4(cur_tile_y, cur_tile_x) = sum(sum(union_4(y_offset:(y_offset+im_height/32-1), x_offset:(x_offset+im_width/32-1))));
        delta_d_residue_tex_4(cur_tile_y, cur_tile_x) = sum(sum(delta_d_4(y_offset:(y_offset+im_height/32-1), x_offset:(x_offset+im_width/32-1))));
        delta_d_residue_tex_4_manual(cur_tile_y, cur_tile_x) = sum(sum(delta_d_4_manual(y_offset:(y_offset+im_height/32-1), x_offset:(x_offset+im_width/32-1))));
    end
end

ok = 1;
if (sum(sum(abs(delta_d_residue_tex_4_manual - delta_d_residue_tex_4) ./abs(delta_d_residue_tex_4) < 1e-6)) ~= ntiles)
    disp('Residue delta_d_4 texture is incorrect!'); ok = 0;
end
if (sum(sum(union_residue_tex_4 == union_correct)) ~= ntiles)
    disp('Residue union_residue_tex_4 texture is incorrect!'); ok = 0;
end
if (sum(sum(intersection_residue_tex_4 == intersection_correct)) ~= ntiles)
    disp('Residue intersection_residue_tex_4 texture is incorrect!'); ok = 0;
end
if (ok == 1)
    disp('residue_texture_4.bin correct');
end

%% Fourth Downsample
[residue_texture_5, delta_d_5, union_5, intersect_5] = loadImageFile('residue_texture_20_tiled.bin', im_width * tile_x / 160, im_height * tile_y / 160, 3, 'float');
delta_d_5 = single(delta_d_5);
% imshow(residue_texture_5./max(max(max(residue_texture_5)))); title('residue_texture_5');

delta_d_5_manual = zeros(im_height * tile_y / 160, im_width * tile_x / 160, 'single');
for v = 1:im_height * tile_x / 160
    for u = 1:im_width * tile_x / 160
        v_range = ((v-1)*160 + 1):((v-1)*160 + 160);
        u_range = ((u-1)*160 + 1):((u-1)*160 + 160);
        delta_d_5_manual(v, u) = sum(sum(delta(v_range, u_range)));
    end
end

delta_d_residue_tex_5_manual = zeros(tile_x, tile_y, 'single');
delta_d_residue_tex_5 = zeros(tile_x, tile_y, 'single');
union_residue_tex_5 = zeros(tile_x, tile_y, 'single');
intersection_residue_tex_5 = zeros(tile_x, tile_y, 'single');
for cur_tile_y = 1:tile_y
    y_offset = (im_height / 160) * (cur_tile_y - 1) + 1;
    for cur_tile_x = 1:tile_x
        x_offset = (im_width / 160) * (cur_tile_x - 1) + 1;
        intersection_residue_tex_5(cur_tile_y, cur_tile_x) = sum(sum(intersect_5(y_offset:(y_offset+im_height/160-1), x_offset:(x_offset+im_width/160-1))));
        union_residue_tex_5(cur_tile_y, cur_tile_x) = sum(sum(union_5(y_offset:(y_offset+im_height/160-1), x_offset:(x_offset+im_width/160-1))));
        delta_d_residue_tex_5(cur_tile_y, cur_tile_x) = sum(sum(delta_d_5(y_offset:(y_offset+im_height/160-1), x_offset:(x_offset+im_width/160-1))));
        delta_d_residue_tex_5_manual(cur_tile_y, cur_tile_x) = sum(sum(delta_d_5_manual(y_offset:(y_offset+im_height/160-1), x_offset:(x_offset+im_width/160-1))));
    end
end

ok = 1;
if (sum(sum(abs(delta_d_residue_tex_5_manual - delta_d_residue_tex_5) ./abs(delta_d_residue_tex_5) < 1e-6)) ~= ntiles)
    disp('Residue delta_d_5 texture is incorrect!'); ok = 0;
end
if (sum(sum(union_residue_tex_5 == union_correct)) ~= ntiles)
    disp('Residue union_residue_tex_5 texture is incorrect!'); ok = 0;
end
if (sum(sum(intersection_residue_tex_5 == intersection_correct)) ~= ntiles)
    disp('Residue intersection_residue_tex_5 texture is incorrect!'); ok = 0;
end
if (ok == 1)
    disp('residue_texture_5.bin correct');
end

final_residues = lambda * (delta_d_residue_tex_5 ./ (union_residue_tex_5 + EPSILON)) + ...
    (1.0 - (2.0.*intersection_residue_tex_5 ./ (intersection_residue_tex_5 + union_residue_tex_5)));
final_residues_correct = lambda * (delta_d_correct ./ (union_correct + EPSILON)) + ...
    (1.0 - (2.0.*intersection_correct ./ (intersection_correct + union_correct)));

if (sum(sum(abs(final_residues_correct - final_residues) ./ abs(final_residues) < 1e-6)) == ntiles)
    disp('Final residues are correct (matlab matches c++)!');
else
    disp('Final residues are INCORRECT (matlab does not match c++)!');
end
