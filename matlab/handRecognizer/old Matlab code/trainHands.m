clear all; close all; clear global; clc;

% compile the required c functions
disp('Compilling source code...');
mex -O generateDecisionTree.c
mex -O evaluateDecisionTree.c

global im_width;
global im_height;

% Load in the Kinect Hand Depth Data
data_dir = './hand_depth_data';
fraction_test_data = 0.05;
down_sample = 4;  % width' = width / down_sample_power & height' = height / down_sample_power --> must be integer!
max_num_images = 2000;  %% There may not be this many in the database
disp('Loading hand data from file...');
[DB, test_DB] = loadHandDepthDataFromDirectory(data_dir, max_num_images,...
  fraction_test_data, down_sample);

% Seed the random number generator
rng(0);

% Pairs of pixels --> Relative depth
WLCoeffs = cell(3,1);
max_offset = int32(128 / down_sample);
WLCoeffs{1} = int32(-max_offset:1:max_offset);  %% u_offset
WLCoeffs{2} = int32(-max_offset:1:max_offset);  %% v_offset
% rel_depths = 0.1*2.^(0:5);
rel_depths = single([0.1 0.25 0.5 0.75 1 2 4 8]);
WLCoeffs{3} = single([-rel_depths(end:-1:1), 0, rel_depths]);

tree_height = int32(18);
num_trees = 1;
WL_samples_per_node = int32(1000);
Forest = cell(num_trees,1);

% Train a decision tree
for cur_tree = 1:num_trees
  disp(['Training decision tree ', num2str(cur_tree), ' of ', num2str(num_trees)]);
  tic;
  DT = [];
  [DT.WL{1}, DT.WL{2}, DT.WL{3}, DT.hist] = ...
    generateDecisionTree(DB.image_data, DB.label_data, ...
    WLCoeffs{1}, WLCoeffs{2}, WLCoeffs{3}, WL_samples_per_node, ...
    tree_height, [im_width, im_height], int32(cur_tree));
  Forest{cur_tree} = DT;
  saveDT('Forest.mat', Forest);
  time = toc;
  disp(['  --> Training time ', num2str(time), ' seconds']);
  disp(['Finished training decision tree ', num2str(cur_tree), ' of ', num2str(num_trees)]);
end

% disp('Decision tree #1 histograms...');
% printDTHistograms(DT.hist);

% Try it on one of the test data images
cur_test_image = 1;
test_image_data = test_DB.image_data(:,cur_test_image);
test_label_data = test_DB.label_data(:,cur_test_image);
labels = evaluateDecisionTree(DT.WL{1}, DT.WL{2}, DT.WL{3}, DT.hist, ...
  test_image_data, [im_width, im_height]);
error = sum(test_label_data ~= labels) / length(labels);
fprintf('total DT error (percent) = %.3f\n', error*100);

% Plot the real data
figure;
plotHandDepthData(test_image_data, test_label_data, im_width, im_height);

% % Plot the labeled data
figure;
plotHandDepthData(test_image_data, labels, im_width, im_height);