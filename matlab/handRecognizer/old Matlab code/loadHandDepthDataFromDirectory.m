function [ DB, test_DB ] = loadHandDepthDataFromDirectory( pathtodirectory, max_num, frac_test_data, down_sample )

global im_width;   im_width = int32(640);
global im_height;  im_height = int32(480);

new_width = im_width / down_sample;
new_height = im_height / down_sample;
if (new_width - floor(new_width) ~= 0 || new_height - floor(new_height) ~= 0)
  error('downsample factor must be integer!');
end

if (frac_test_data < 0)
    error('frac_test_data < 0 --> Please make it [0, 1)');
end
if (frac_test_data >= 1)
    error('frac_test_data >= 1 --> Please make it [0, 1)');
end

if (pathtodirectory(end) ~= '/')
    pathtodirectory = strcat(pathtodirectory,'/');
end

listing = dir(strcat(pathtodirectory,'hands_*.bin'));
N = length(listing);

if (N > max_num)
    N = max_num;
end

if (frac_test_data ~= 0)
    stride_DB_test = round(N / (N * frac_test_data));
    indices_DB_test = 1:stride_DB_test:N;
    length_DB_test = length(indices_DB_test);
    legnth_DB = N - length_DB_test;
else
    length_DB_test = 0;
    legnth_DB = N;
end

DB = [];
DB.filenames = cell(legnth_DB, 1);
DB.image_data = zeros(new_width*new_height, legnth_DB, 'uint16');
DB.label_data = zeros(new_width*new_height, legnth_DB, 'uint16');

if (frac_test_data ~= 0)
    test_DB = [];
    test_DB.filenames = cell(length_DB_test, 1);
    test_DB.image_data = zeros(new_width*new_height, length_DB_test, 'uint16');
    test_DB.label_data = zeros(new_width*new_height, length_DB_test, 'uint16');
else
    test_DB = [];
    test_DB.filenames = [];
    test_DB.image_data = [];
    test_DB.label_data = [];
end

% remove some data from the training data as test data.
% The training data is captured as a live video stream, so grab the data
% at regular intervals --> Which results in test data that is a reasonable
% subset of the hand poses.

i_DB = 1;
i_test_DB = 1;
for i = 1:N
    if (frac_test_data ~= 0 && (mod(i, stride_DB_test) == 1))
        % Node belongs to the test DB
        test_DB.filenames{i_test_DB} = [pathtodirectory,listing(i).name];
        [depth_data, mask_data] = loadHandDepthData(test_DB.filenames{i_test_DB});
        if (down_sample ~= 1)
          depth_data = downsampleImage(depth_data, im_width, im_height, down_sample);
          mask_data = downsampleImage(mask_data, im_width, im_height, down_sample);
        end
        test_DB.image_data(:, i_test_DB) = depth_data;
        test_DB.label_data(:, i_test_DB) = mask_data;
        i_test_DB = i_test_DB + 1;
    else
        % Node belongs to the training DB
        DB.filenames{i_DB} = [pathtodirectory,listing(i).name];
        [depth_data, mask_data] = loadHandDepthData(DB.filenames{i_DB});
        if (down_sample ~= 1)
          depth_data = downsampleImage(depth_data, im_width, im_height, down_sample);
          mask_data = downsampleImage(mask_data, im_width, im_height, down_sample);
        end
        DB.image_data(:, i_DB) = depth_data;
        DB.label_data(:, i_DB) = mask_data;   
        i_DB = i_DB + 1;
    end
end

im_width = new_width;
im_height = new_height;

end

