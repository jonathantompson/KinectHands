function [ ] = plotHandDepthData( depth_data, mask_data, width, height )

% plot the mask
im_mask = reshape(mask_data, width, height)';
border_pixels = find(im_mask == 2);
border_pixels_neg = ones(height, width);
border_pixels_neg(border_pixels) = 0;
im_mask(border_pixels) = 0;
im_mask_neg = double(~im_mask);

% plot the depth
im_depth_data = double(reshape(depth_data, width, height)');
% rescale 
max_depth = max(max(im_depth_data));
min_depth = min(min(im_depth_data));
im_depth_data = (im_depth_data - min_depth) ./ (max_depth - min_depth);
im_depth_data = im_depth_data*(-1) + 1;

Argb = cat(3, im_depth_data.*border_pixels_neg, im_depth_data.*im_mask_neg.*border_pixels_neg, im_depth_data.*im_mask_neg);
imshow(Argb);
 
end

