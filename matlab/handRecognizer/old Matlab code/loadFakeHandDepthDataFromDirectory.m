function [ DB, test_DB ] = loadFakeHandDepthDataFromDirectory(N_hand_images, frac_test_data )
    
    global im_width;
    global im_height;

    rng(0);
    if (frac_test_data ~= 0)
        stride_DB_test = N_hand_images / (N_hand_images * frac_test_data);
        indices_DB_test = 1:stride_DB_test:N_hand_images;
        length_DB_test = length(indices_DB_test);
        legnth_DB = N_hand_images - length_DB_test;
    else 
        length_DB_test = 0;
        legnth_DB = N_hand_images;
    end
    
    DB = [];
    DB.filenames = cell(legnth_DB, 1);
    DB.image_data = zeros(im_width*im_height, legnth_DB, 'uint16');
    DB.label_data = zeros(im_width*im_height, legnth_DB, 'uint16');

    if (frac_test_data ~= 0)
        test_DB = [];
        test_DB.filenames = cell(length_DB_test, 1);
        test_DB.image_data = zeros(im_width*im_height, length_DB_test, 'uint16');
        test_DB.label_data = zeros(im_width*im_height, length_DB_test, 'uint16');
    else
        test_DB = []; 
        test_DB.filenames = [];
        test_DB.image_data = [];
        test_DB.label_data = [];
    end    
    
    i_DB = 1;
    i_test_DB = 1;
    for i = 1:N_hand_images
        if (frac_test_data ~= 0 && mod(i, stride_DB_test) == 1) 
            % Node belongs to the test DB
            test_DB.filenames{i_test_DB} = ['data_',num2str(i)];
            [depth_data, mask_data] = loadFakeHandDepthData();
            test_DB.image_data(:, i_test_DB) = depth_data;
            test_DB.label_data(:, i_test_DB) = mask_data;
            i_test_DB = i_test_DB + 1;
        else
            % Node belongs to the training DB
            DB.filenames{i_DB} = ['data_',num2str(i)];
            [depth_data, mask_data] = loadFakeHandDepthData();
            DB.image_data(:, i_DB) = depth_data;
            DB.label_data(:, i_DB) = mask_data;   
            i_DB = i_DB + 1;
        end
    end
end