function [ im_data, r, g, b, a ] = loadImageFile( filename, width, height, channels, type )

file = fopen(filename);
im_data = fread(file, width * height * channels, type);
fclose(file);

r = [];
g = [];
b = [];
a = [];

switch channels
  case 1
    r = reshape(im_data, width, height )';
    im_data = r;
  case 2
    r = reshape(im_data(1:2:end-1), width, height )';
    g = reshape(im_data(2:2:end), width, height )';
    im_data = cat(2, r, g);
  case 3
    r = reshape(im_data(1:3:end-2), width, height )';
    g = reshape(im_data(2:3:end-1), width, height )';
    b = reshape(im_data(3:3:end), width, height )';
    im_data = cat(3, r, g, b);
  case 4
    r = reshape(im_data(1:4:end-3), width, height )';
    g = reshape(im_data(2:4:end-2), width, height )';
    b = reshape(im_data(3:4:end-1), width, height )';
    a = reshape(im_data(4:4:end), width, height )';
    im_data = cat(4, r, g, b, a);
  otherwise
    error('loadImageFile: Need 1 <= channels <= 4');
end

end

