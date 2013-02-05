function [ DB ] = extractRawHandContoursFromDirectory( pathtodirectory )

global shape_context_n_bins;
shape_context_n_bins = 16;
global shape_context_n_pts;
shape_context_n_pts = 100;

if (pathtodirectory(end) ~= '/')
    pathtodirectory = strcat(pathtodirectory,'/');
end

listing = dir(strcat(pathtodirectory,'contour_hands_*.bin'));
N = length(listing);

DB = [];
DB.contour_sizes = cell(1,N);
DB.contour_data = cell(1,N);
DB.shape_contexts = cell(1,N);
for i = 1:N
    filename = [pathtodirectory,listing(i).name];
    fprintf('%g of %g: %s\n',i,N,filename);
    [contour_sizes, contour_data] = extractRawHandContoursFromFile(filename);
    DB.contour_sizes{i} = contour_sizes;
    DB.contour_data{i} = contour_data;
end

fprintf('Creating shape contexts\n');
for i = 1:N
    filename = [pathtodirectory,listing(i).name];
    fprintf('hand_context %g of %g: %s\n',i,N,filename);
    [max_contour_length, max_contour_index] = max(DB.contour_sizes{i});
    contour_data = DB.contour_data{i};
    xyz_contour_data = contour_data{max_contour_index};
    DB.shape_contexts{i} = getSimpleShapeContextFrom2DContour( xyz_contour_data(:,1), ...
        xyz_contour_data(:,2), shape_context_n_pts, shape_context_n_bins );
end

end