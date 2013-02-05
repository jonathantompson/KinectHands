clear all; close all; clear global; clc;

% training_data_dir = [getenv('HOME'), '/Desktop/hand_data'];
training_data_dir = './hand_data';
Contours_DB = extractRawHandContoursFromDirectory(training_data_dir);
Contours_DB_size = length(Contours_DB.shape_contexts);
% HandData_DB = extractRawHandsFromDirectoryJT(training_data_dir);

% test_data_dir = [getenv('HOME'), '/Desktop/hand_data_tests'];
test_data_dir = './hand_data_testing';
Contours_test_DB = extractRawHandContoursFromDirectory(test_data_dir);
Contours_test_DB_size = length(Contours_test_DB.shape_contexts);

%% Pick a contour from random from the Contours_test_DB and match it to
% the training data
rng(10);
ind = floor(rand(1)*(Contours_test_DB_size-1)) + 1;
% ind = 1;
cur_shape_context = Contours_test_DB.shape_contexts{ind};
cur_contours = Contours_test_DB.contour_data{ind};
[cur_contour_length, cur_contour_index] = max(Contours_test_DB.contour_sizes{ind});
cur_contour = cur_contours{cur_contour_index};

% Check against each shape context in the database
errors = zeros(1, Contours_DB_size);
for m = 1:Contours_DB_size
    disp(['Checking against shape context ', num2str(m), ' of ',num2str(Contours_DB_size)]);
    cur_training_shape_context = Contours_DB.shape_contexts{m};
    delta = cur_shape_context - cur_training_shape_context;
    errors(m) = sum(sum(delta .*delta));
end

% Plot the top 8 matches
[M, I] = sort(errors);
figure; subplot(5, 5, 1);
plotContour(cur_contour);
for m = 1:24
    training_ind = I(m);
    cur_training_contours = Contours_DB.contour_data{training_ind};
    [cur_training_contour_length, cur_training_contour_index] = max(Contours_DB.contour_sizes{training_ind});
    cur_training_contour = cur_training_contours{cur_training_contour_index};
    subplot(5,5,m+1);
    plotContour(cur_training_contour);
end