function [ down_im ] = downsample_by_2( im )

width = length(im(1,:));
height = length(im(:,1));

if ((height/2 - floor(height/2))~=0) 
  error('ERROR: downsample_by_2() - height must be divisible by 2!');
end
if ((width/2 - floor(width/2))~=0) 
  error('ERROR: downsample_by_2() - width must be divisible by 2!');
end

down_im = zeros(height/2, width/2);
for v = 1:height/2
  for u = 1:width/2
    v_src = 2*v - 1;
    u_src = 2*u - 1;
    down_im(v,u) = 0.25*(im(v_src, u_src) + im(v_src+1, u_src) + ...
                         im(v_src, u_src+1) + im(v_src+1, u_src+1));
  end
end

end

