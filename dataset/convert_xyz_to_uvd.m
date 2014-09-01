function [ uvd ] = convert_xyz_to_uvd( xyz )
halfResX = 640/2;
halfResY = 480/2;
coeffX = 588.036865;
coeffY = 587.075073;

uvd = zeros(size(xyz));
uvd(:,:,1) = coeffX * xyz(:,:,1) ./ xyz(:,:,3) + halfResX;
uvd(:,:,2) = halfResY - coeffY * xyz(:,:,2) ./ xyz(:,:,3);
uvd(:,:,3) = xyz(:,:,3);

end

