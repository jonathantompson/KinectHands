function [ output_image, width_new, height_new ] = addImageBorder( image, border_size, border_value, width, height )

width_new = width + 2 * border_size;
height_new = height + 2 * border_size;

output_image = ones(width_new * height_new, 1, class(image(1)))* border_value;

for v = 1:height
    for u = 1:width
        index_orig = (v-1)*width + u;
        index_new = (v+border_size - 1) * width_new + (u + border_size);
        output_image(index_new) = image(index_orig);
    end
end

end

