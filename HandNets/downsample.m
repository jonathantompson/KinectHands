function [ down_im ] = downsample( im, downsample )

width = length(im(1,:));
height = length(im(:,1));

if ((height/downsample - floor(height/downsample))~=0) 
  error('ERROR: downsample_by_2() - height must be divisible by 2!');
end
if ((width/downsample - floor(width/downsample))~=0) 
  error('ERROR: downsample_by_2() - width must be divisible by 2!');
end

down_im = zeros(height/downsample, width/downsample, class(im));
scale = downsample*downsample;
for v = 1:height/downsample
  for u = 1:width/downsample
    down_im(v,u) = 0;
    src_v = downsample*(v-1) + 1;
    src_u = downsample*(u-1) + 1;
    for voff = src_v:src_v+downsample-1
      for uoff = src_u:src_u+downsample-1
        down_im(v,u) = down_im(v,u) + im(voff, uoff);
      end
    end
    down_im(v,u) = down_im(v,u) / scale;
  end
end

end

