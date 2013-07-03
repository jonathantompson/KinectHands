function [ im_filtered ] = filter_zero_bndry( im, kernel1d )

kernel_center = length(kernel1d)/2 + 0.5;
kernel_rad = (length(kernel1d) - 1)/2;
w = length(im(1,:));
h = length(im(:,1));

% Filter horizontally
intermediate_res = zeros(h, w, class(im));
for v = 1:h
  for u = 1:w
    for filt = -kernel_rad:1:kernel_rad
      uoff = u + filt;
      if ((uoff >=1) && (uoff <= w))
        intermediate_res(v, u) = intermediate_res(v, u) + ...
          (kernel1d(filt + kernel_rad + 1) * im(v, uoff));
      end
    end
  end
end

% Filter vertically
im_filtered = zeros(h, w, class(im));
for v = 1:h
  for u = 1:w
    for filt = -kernel_rad:1:kernel_rad
      voff = v + filt;
      if ((voff >=1) && (voff <= h))
        im_filtered(v, u) = im_filtered(v, u) + ...
          (kernel1d(filt + kernel_rad + 1) * intermediate_res(voff, u));
      end
    end
  end
end

end

