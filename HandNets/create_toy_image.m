function [ im ] = create_toy_image( p_mat, quad_vert, hand_pos, hand_size, ...
  width, height)

im = zeros(480, 640, 'int16');

% Transform vertices into screen space
model_mat = zeros(4, 4, 'single');  % Scale + translation
model_mat(1, 1) = hand_size;
model_mat(2, 2) = hand_size;
model_mat(3, 3) = hand_size;
model_mat(1, 4) = hand_pos(1);
model_mat(2, 4) = hand_pos(2);
model_mat(3, 4) = hand_pos(3);
model_mat(4, 4) = 1;

pm = p_mat * model_mat;

n_vert = length(quad_vert(1,:));
vert_ndc = zeros(4, n_vert, 'single');
vert_screen = zeros(2, n_vert, 'single');

for i = 1:n_vert
  vert_ndc(:,i) = pm * quad_vert(:,i);
  vert_ndc(:,i) = vert_ndc(:,i) / vert_ndc(4,i);  % [-1, 1]
  vert_screen(1,i) = (vert_ndc(1,i)+1) * width / 2;
  vert_screen(2,i) = (vert_ndc(2,i)+1) * height / 2;
end

% % Full intersection test
% Now rasterize each quad seperately --> This is a shitty rasterization
% method.  For each pixel I'll just do a line test with each of the
% quad edges --> Line equations are a^Tx = b, where a is the line normal
% a = zeros(2, n_vert, 'single');
% b = zeros(1, n_vert, 'single');
% rot_mat = [cos(pi/2) -sin(pi/2); sin(pi/2) cos(pi/2)];  %% 90 degrees left
% for i = 1:4
%   i1 = i;
%   if i == 4; i2 = 1; else i2 = i+1; end
%   edge_vec = vert_screen(:, i2) - vert_screen(:, i1);
%   edge_vec = edge_vec / sqrt(edge_vec' * edge_vec);  % normalize
%   % Find orthogonal vertex
%   a(:,i) = rot_mat * edge_vec;
%   b(1,i) = a(:,i)' * vert_screen(:, i1);
% end
% for v = 1:height
%   for u = 1:width
%     pix = [u; v];
%     % Perform plane tests
%     in_plane = true;
%     cur_edge = 1;
%     while (in_plane && cur_edge <= 4)
%       in_plane = in_plane && ((a(:,cur_edge)' * pix) <= b(1,cur_edge));
%       cur_edge = cur_edge + 1;
%     end
%     
%     if (in_plane)
%       im(v, u) = -hand_pos(3);
%     else
%       im(v, u) = 2001;
%     end
%   end
% end

% Intersection test assuming perfect square in screen space
y_vals = 1:height;
inside_y =(y_vals <= vert_screen(2,2)) + (y_vals >= vert_screen(2,1)) == 2;
x_vals = 1:width;
inside_x =(x_vals >= vert_screen(1,2)) + (x_vals <= vert_screen(1,3)) == 2;
for v = 1:height
  if (inside_y(v)) 
    im(v, :) = (-inside_x + 1) * (2001 - (-hand_pos(3))) + -hand_pos(3);
  else
    im(v, :) = 2001;
  end
end

