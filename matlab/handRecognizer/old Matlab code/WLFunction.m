function [ output ] = WLFunction( data, indices, WLCoeff )

global im_width;
global im_height;
global max_offset;

% data:            is the input image --> as a 1D array
% uv:              is the uv index of the test pixel
% WLCoefficients:  are the current coefficients of this weak lerner
%                  WLCoeff(0) - u offset
%                  WLCoeff(1) - v offset
%                  WLCoeff(2) - threshold
% output:          binary output decision (0 = left subtree, 1 = right)

indices_offset = indices + WLCoeff(2) * (im_width + 2*max_offset) + WLCoeff(1);
delta_vals = double(data(indices_offset) - data(indices)) ./ double(data(indices));
output = delta_vals >= WLCoeff(3);


% % get the j pixel using the u and v offsets
% u_off = uv(1) + WLCoeff(1);
% v_off = uv(2) + WLCoeff(2);
% if (u_off > im_width || u_off < 1 || ... 
%     v_off > im_height || v_off < 1)
%     output = 1;  % ith pixel is always infront of pixels off the image
% else 
%     data_i = ((uv(2) - 1) * im_width) + uv(1);
%     if (data(data_i) == 0)
%         output = 1;  % zero pixels are always behind anything else
%     else
%         data_j = ((u_off - 1) * im_width) + v_off;
%         delta_val = (data(data_j) - data(data_i)) / data(data_i);
%         if (delta_val >= WLCoeff(3))
%             output = 1;
%         else
%             output = 0;
%         end
%     end
% end

end

