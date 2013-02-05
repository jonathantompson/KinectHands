function [contour_sizes, contour_data] = extractRawHandContoursFromFile(pathtofile)

    file = fopen(pathtofile);
    num_contours = fread(file, 1, 'uint32');
    contour_sizes = fread(file, num_contours, 'uint32');
    contour_data = cell(1,num_contours);
    
    % Load in the raw data
    for i = 1:num_contours
        cur_data = fread(file, contour_sizes(i)*3, 'float');
        cur_data = reshape(cur_data,3,contour_sizes(i))';
        contour_data{i} = cur_data;
    end
    fclose(file);
end