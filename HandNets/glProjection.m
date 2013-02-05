function [ mat ] = glProjection( znear, zfar, fov_deg, screen_width, ...
  screen_height )
mat = zeros(4, 4, 'single');

aspect = screen_width / screen_height;

xymax = znear * tan(fov_deg * pi / 360);
ymin = -xymax;
xmin = -xymax;

width = xymax - xmin;
height = xymax - ymin;

depth = zfar - znear;
q = -(zfar + znear) / depth;
qn = -2 * (zfar * znear) / depth;

w = 2 * znear / width;
w = w / aspect;
h = 2 * znear / height;

% First column
mat(1, 1)  = w;
mat(2, 1)  = 0;
mat(3, 1)  = 0;
mat(4, 1)  = 0;

% Second column
mat(1, 2)  = 0;
mat(2, 2)  = h;
mat(3, 2)  = 0;
mat(4, 2)  = 0;

% Third column
mat(1, 3)  = 0;
mat(2, 3)  = 0;
mat(3, 3) = q;
mat(4, 3) = -1;

% Fourth column
mat(1, 4) = 0;
mat(2, 4) = 0;
mat(3, 4) = qn;
mat(4, 4) = 0;

end

