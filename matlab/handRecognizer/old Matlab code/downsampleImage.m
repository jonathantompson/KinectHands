function [ return_image ] = downsampleImage( image, width, height, down_sample )

if (down_sample > 1)
  new_width = width / down_sample;
  new_height = height / down_sample;
  if (new_width - floor(new_width) ~= 0 || new_height - floor(new_height) ~= 0)
    error('downsampleImage() - Cannot downsample fractional values!');
  end
  image = reshape(image, width, height);
  return_image = imresize(image, 1/down_sample);
  return_image = reshape(return_image, new_width*new_height, 1);
else
  return_image = image;
end

end

