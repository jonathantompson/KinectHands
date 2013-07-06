clearvars; clc; clear global; close all; format shortG;
addpath('./export_fig/');

%% Kinect Plots
filename = 'kinect_texture.bin';
width = 640;
height = 480;
channels = 1;
type = 'int16';
kinect = single(loadImageFile(filename, width, height, channels, type));

filename = 'synth_texture.bin';
type = 'single';
synthetic = single(loadImageFile(filename, width, height, channels, type));

filename = 'residue_texture.bin';
type = 'single';
residue = single(loadImageFile(filename, width, height, channels, type));

% Calculate the center of mass of the sythetic
com = [0,0];
min_max_depth = [inf, -inf];
num_pts = 0;
for v = 1:480
    for u = 1:640
        if (synthetic(v, u) ~= 0) 
            com = com + [u,v];
            num_pts = num_pts + 1;
            if (min_max_depth(1) > synthetic(v, u))
                min_max_depth(1) = synthetic(v, u);
            end
            if (min_max_depth(2) < synthetic(v, u))
                min_max_depth(2) = synthetic(v, u);
            end            
        end
    end
end
com = com / num_pts;

crop_size_horiz = 85;
crop_size_vert = 128;
bounds_u = [max(1, round(com(1)) - crop_size_horiz+10) min(640, round(com(1)) + crop_size_horiz)];
bounds_v = [max(1, round(com(2)) - crop_size_vert) min(480, round(com(2)) + crop_size_vert-10)];
kinect_cropped = kinect(bounds_v(1):bounds_v(2), bounds_u(1):bounds_u(2));
synthetic_cropped = synthetic(bounds_v(1):bounds_v(2), bounds_u(1):bounds_u(2));
residue_cropped = residue(bounds_v(1):bounds_v(2), bounds_u(1):bounds_u(2));

background = 2001;

synthetic_cropped(find(synthetic_cropped == 0)) = min_max_depth(2);
% residue_cropped(find(residue_cropped == 30)) = 0;

kinect_cropped = (kinect_cropped - min_max_depth(1))/ (min_max_depth(2)-min_max_depth(1));
synthetic_cropped = (synthetic_cropped - min_max_depth(1))/ (min_max_depth(2)-min_max_depth(1));

figure;
imshow(kinect_cropped, 'Border', 'tight');
set(gcf, 'Color', 'w');
export_fig PrimeSenseDepth.png -m2 -painters -a4 -nocrop
figure;
imshow(synthetic_cropped, 'Border', 'tight');
set(gcf, 'Color', 'w');
export_fig SyntheticDepth.png -m2 -painters -a4 -nocrop
figure;
imshow((residue_cropped - min(min(residue_cropped)))/ (0.75*(max(max(residue_cropped))-min(min(residue_cropped)))), 'Border', 'tight');
set(gcf, 'Color', 'w');
export_fig Residue.png -m2 -painters -a4 -nocrop

%% Decision tree plots
dt_file = '../../HandForests/results.xlsx';
num_trees_test = xlsread(dt_file, 'TOG_num_trees_test');
num_trees_train = xlsread(dt_file, 'TOG_num_trees_train');
height_test = xlsread(dt_file, 'TOG_height_test');
height_train = xlsread(dt_file, 'TOG_height_train');
headings = ['Height or Number of Trees', 'Number Incorrect Pixels', 'Number Correct Pixels', 'Number of Hand Pixels', 'Number False Positives', 'Number False Negatives'];

figure;
plot(num_trees_test(:,1), 100 * (num_trees_test(:,2) ./ num_trees_test(:,4)), 'r', 'LineWidth', 1.5); hold on;
plot(num_trees_train(:,1), 100 * (num_trees_train(:,2) ./ num_trees_train(:,4)), 'b', 'LineWidth', 1.5);
xlabel('Number of Trees');
ylabel('Percentage Error');
legend({'Test Set', 'Training Set'}, 'Location', 'NorthEast');
grid on;
makePlotNice();
set(gca, 'Position', [.7 .7 4.5 3.125]); 
export_fig error_vs_num_trees.png -m2 -painters -a4 -nocrop

figure;
plot(height_test(:,1), 100 * (height_test(:,2) ./ height_test(:,4)), 'r', 'LineWidth', 1.5); hold on;
plot(height_train(:,1), 100 * (height_train(:,2) ./ height_train(:,4)), 'b', 'LineWidth', 1.5);
xlabel('Height of Trees');
ylabel('Percentage Error');
legend({'Test Set', 'Training Set'}, 'Location', 'NorthEast');
set(gcf, 'Color', 'w');
grid on;
makePlotNice();
export_fig error_vs_height.png -m2 -painters -a4 -nocrop

%% Convnet Plots
cnn_file = '../../HandNets/results.xls';
epoch_test = xlsread(cnn_file, 'MSE Errors 8','B4:B318');
error_test = xlsread(cnn_file, 'MSE Errors 8','D4:D318');
epoch_train = xlsread(cnn_file, 'MSE Errors 8','AA4:AA317');
error_train = xlsread(cnn_file, 'MSE Errors 8','AC4:AC317');

figure;
loglog(epoch_test, error_test, 'r', 'LineWidth', 1.5); hold on;
loglog(epoch_train, error_train, 'b', 'LineWidth', 1.5);
axis([min(epoch_test) max(epoch_test) min(error_train) max(error_test)]);
xlabel('Learning Epoch');
ylabel('MSE Error');
legend({'Test Set', 'Training Set'}, 'Location', 'NorthEast');
grid on;
makePlotNice();
set(gca, 'Position', [.8 .75 4.5 3.125]); 
export_fig conv_learning_graph.png -m2 -painters -a4 -nocrop
