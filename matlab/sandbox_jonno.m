close all; clear all; clc; clear global;

%% Just load one of the contours from file and visualize it
% test_file = '/Users/jonathantompson/Desktop/hand_data/contour_hands_405904816941425.bin';
test_file = './hand_data/contour_hands_14600761612.bin';
[contour_sizes, contour_data] = extractRawHandContoursFromFile(test_file);

%% 3D plot
% figure;
% for i = 1:length(contour_sizes)
%     xyz_data = contour_data{i};
%     plot3(xyz_data(:,1), xyz_data(:,2), xyz_data(:,3), 'Color', getColor(i));
%     hold on;
%     plot3([xyz_data(end,1) xyz_data(1,1)], ...
%         [xyz_data(end,2) xyz_data(1,2)], ...
%         [xyz_data(end,3) xyz_data(1,3)], 'Color', getColor(i));
% end
% grid on;

%% 2D projection plot (just throw away the z value)
% figure;
% for i = 1:length(contour_sizes)
%     xyz_data = contour_data{i};
%     plot(xyz_data(:,1), xyz_data(:,2), 'Color', getColor(i)); hold on;
%     plot([xyz_data(end,1) xyz_data(1,1)], ...
%         [xyz_data(end,2) xyz_data(1,2)], 'Color', getColor(i));
% end

%% Calculate the shape context from the longest contour
n_pts = 200;
[max_contour_size max_contour_size_index] = max(contour_sizes);
disp(['Number of points in the original contour = ',num2str(max_contour_size)]);
xyz_data = contour_data{max_contour_size_index};
shape_context_n_bins = 16;
[shape_context, samples] = getSimpleShapeContextFrom2DContour(xyz_data(:,1), xyz_data(:,2), n_pts, shape_context_n_bins);
figure;
plot(xyz_data(:,1), xyz_data(:,2), 'Color', getColor(1)); hold on;
plot([xyz_data(end,1) xyz_data(1,1)], [xyz_data(end,2) xyz_data(1,2)], 'Color', getColor(1));
plot(samples(:,1), samples(:,2), 'o', 'Color', getColor(2)); hold on;
plot([samples(end,1) samples(1,1)], [samples(end,2) samples(1,2)], 'o', 'Color', getColor(2));
axis([-150 150 -150 150]);
figure;
h = pcolor(1:shape_context_n_bins,1:shape_context_n_bins,shape_context); 
set(h, 'edgecolor','black');
% shading(gca,'interp'); 
colorbar;
disp(['Number of shape context samples = ',num2str(sum(sum(shape_context)))]);