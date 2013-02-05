function [ ret_DB, ret_test_DB ] = addImageBorderToDB( DB, test_DB, ...
    border_size, border_value_depth, border_value_label, width, height )

DB_length = length(DB.image_data(1,:));
ret_DB = [];
ret_DB.image_data = zeros((width+2*border_size)*(height+2*border_size), DB_length, 'uint16');
ret_DB.label_data = zeros((width+2*border_size)*(height+2*border_size), DB_length, 'uint16');

test_DB_length = length(test_DB.image_data(1,:));
ret_test_DB = [];
ret_test_DB.image_data = zeros((width+2*border_size)*(height+2*border_size), test_DB_length, 'uint16');
ret_test_DB.label_data = zeros((width+2*border_size)*(height+2*border_size), test_DB_length, 'uint16');

for i = 1:DB_length
    ret_DB.image_data(:,i) = addImageBorder(DB.image_data(:,i), border_size, border_value_depth, width, height);
    ret_DB.label_data(:,i) = addImageBorder(DB.label_data(:,i), border_size, border_value_label, width, height);
end
for i = 1:test_DB_length
    ret_test_DB.image_data(:,i) = addImageBorder(test_DB.image_data(:,i), border_size, border_value_depth, width, height);
    ret_test_DB.label_data(:,i) = addImageBorder(test_DB.label_data(:,i), border_size, border_value_label, width, height);
end


end

