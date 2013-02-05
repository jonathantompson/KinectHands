function [depth_data, mask_data] = loadHandDepthData( pathtofile )

    width = 640;
    height = 480;

    % Load in the raw data
    file = fopen(pathtofile);

    depth_data = fread(file, width*height, 'int16=>int16');
    fclose(file);

    % Check the depth data
    mask_data = zeros(width*height,1,'uint16');
    mask_data(find(depth_data < 0)) = uint16(1);
    depth_data = abs(depth_data);
end