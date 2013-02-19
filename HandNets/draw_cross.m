function [ ] = draw_cross( xy, rad, size )

xy = xy .* [size(1) size(2)];

% horiz
minx = max(xy(1)-rad, 1);
maxx = min(xy(1)+rad, size(1));
line([minx, maxx],[xy(2), xy(2)]);

% vert
miny = max(xy(2)-rad, 1);
maxy = min(xy(2)+rad, size(2));
line([xy(1), xy(1)],[miny, maxy]);

end

