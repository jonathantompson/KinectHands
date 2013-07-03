function [ dst ] = integral_image( src, width, height)
dst = src;

% Sum horizontally
for x = 2:width
  for y = 1:height
    i1 = (x-1);
    i2 = x;
    dst(y, i2) = dst(y, i1) + dst(y, i2);
  end
end

% Sum vertically
for x = 1:width
  for y = 2:height
    i1 = (y-1);
    i2 = y;
    dst(i2, x) = dst(i1, x) + dst(i2, x);
  end
end

end