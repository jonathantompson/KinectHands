clearvars; close all; clear textprogressbar; clc;

root_dir = 'F:\hand_data\';

% Compile the mex file that will allow us to load in the compressed depth +
% rgb images from the database
display('Compiling the c++ source...');
mex -largeArrayDims -I.\c_src\ .\c_src\load_depth_image.cpp .\c_src\fastlz.c

% Get a list of the video sequences (they are just the sub directories)
dirs = dir(root_dir);
dirs = {dirs(find([dirs(:).isdir] == 1)).name};
dirs(ismember(dirs,{'.','..'})) = [];  % Remove '.' and '..' directories

% Load the depth, rgb and coeff files
depth = cell(1, length(dirs));
rgb = cell(1, length(dirs));
coeff = cell(1, length(dirs));
for i = 1:length(dirs)
  % Get the right hand coeff files (I didn't fit a left hand, the coeff are all 
  % zero)
  coeff_files = dir([root_dir dirs{i} '/coeffr*']);
  [~, index] = sort({coeff_files.name});
  coeff_files = coeff_files(index);  % sort by name just in case it isn't
  
  % Initialize the coeff
  num_coeff = coeff_files(1).bytes / 4;
  coeff{i} = zeros(length(coeff_files), num_coeff, 'single');
  textprogressbar('Loading the coefficient files: ');
  for f = 1:length(coeff_files)
    textprogressbar(100 * f / length(coeff_files));
    fileID = fopen([root_dir dirs{i} '/' coeff_files(f).name], 'r');
    coeff{i}(f, :) = fread(fileID, num_coeff, 'single');
    fclose(fileID);
  end
  textprogressbar(' done');
  
  textprogressbar('Loading the rgb + depth files (from first camera only): ');
  im_files = dir([root_dir dirs{i} '/hands0*']);
  [~, index] = sort({im_files.name});
  im_files = im_files(index);  % sort by name just in case it isn't
  rgb{i} = zeros(length(im_files), 480, 640, 3, 'uint8');
  depth{i} = zeros(length(im_files), 480, 640, 'uint16');
  for f = 1:length(im_files)
    textprogressbar(100 * f / length(im_files));
    % The depth and rgb images are stored in a compressed file format.
    % In retrospect I should have used zip on the entire dataset, since the
    % compression overhead of decompressing a single file at a time is very
    % slow.  In any case, we must call our cpp file, which will call the
    % decompression library.  This is pretty slow.
    [cur_depth, cur_rgb] = ...
      load_depth_image([root_dir dirs{i} '/' im_files(f).name]);
    rgb{i}(f,:,:,:) = cur_rgb;
    depth{i}(f,:,:,:) = cur_depth;
  end
  textprogressbar(' done');
end

% Visualize some examples
iexamples = randi([1 length(depth{1})], 1, 5);
for i = 1:length(iexamples)
  figure;
  cur_depth = squeeze(double(depth{1}(iexamples(i),:,:)));
  max_depth = max(cur_depth(:));
  cur_depth(find(cur_depth <= 0)) = max_depth;
  cur_depth = cur_depth - min(cur_depth(:));
  cur_depth = cur_depth / max(cur_depth(:));
  imshow(cur_depth);
end

