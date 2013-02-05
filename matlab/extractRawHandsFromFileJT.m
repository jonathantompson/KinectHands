function [xyz, rgb, depth_data, mask_data] = extractrawhandsfromfile(pathtofile)

    width = 640;
    height = 480;

    % Load in the raw data
    file = fopen(pathtofile);
    xyz_data = fread(file, width*height*3, 'float');
    rgb_data = fread(file, width*height*3, 'char');
    mask_data = fread(file, width*height, 'char');  % 0-no hand, 1-LHand, 2-RHand, 3-both hands
    depth_data = fread(file, width*height, 'uint16');
    fclose(file);

    % Check the depth data
    x = reshape(xyz_data(1:3:end-2), width, height )';
    y = reshape(xyz_data(2:3:end-1), width, height )';
    z = reshape(xyz_data(3:3:end), width, height )';
    mask_data = reshape(mask_data, width, height )';
    depth_data = reshape(depth_data, width, height)';

    % Represent raw double data as rgb
    r = reshape(rgb_data(1:3:end-2), width, height )';
    g = reshape(rgb_data(2:3:end-1), width, height )';
    b = reshape(rgb_data(3:3:end), width, height )';
    rgb = cat(3,double(r),double(g),double(b)) / 255.0;
    xyz = cat(3,x,y,z);
end