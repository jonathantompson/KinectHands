function [depth_data, mask_data] = loadFakeHandDepthData( )
    
    global im_width;
    global im_height;
    
    width = double(im_width);
    height = double(im_height);    
    
    % Pick a random center (in the image)
    
    u_center = floor((width-1)*rand(1))+1;
    v_center = floor((height-1)*rand(1))+1;
    sigma_u = 100;  % 50 pixel sigma
    sigma_v = 200;  % 100 pixel sigma
    Amplitude = -100;
    Offset = 110;  %% We want NO zero values during testing

    % Make the depth data an offset and scaled gaussian
    % http://en.wikipedia.org/wiki/Gaussian_function
    
    % depth_data = zeros(im_width*im_height,1,'uint16');
    
    % The filter is seperable
    u = 1:width;
    u_gauss = exp(-(u-u_center).^2./(2*sigma_u^2));
    v = 1:height;
    v_gauss = exp(-(v-v_center).^2./(2*sigma_v^2));
    data = u_gauss' * v_gauss;  % 640 x 480
    data = reshape(data, width*height, 1);
    depth_data = uint16(floor(Offset + Amplitude * data));
    
    % Threshold the depth data to define the hand
    mask_data = zeros(width*height, 1, 'uint16');
    mask_data(find(depth_data < 20)) = uint16(1);
end