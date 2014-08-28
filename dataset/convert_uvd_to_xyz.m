function [ xyz ] = convert_uvd_to_xyz( uvd )
xRes = 640;
yRes = 480;
xzFactor = 1.08836710;
yzFactor = 0.817612648;

normalizedX = double(uvd(:,:,1)) / xRes - 0.5;
normalizedY = 0.5 - double(uvd(:,:,2)) / yRes;

xyz = zeros(size(uvd));
xyz(:,:,3) = double(uvd(:,:,3));
xyz(:,:,1) = normalizedX .* xyz(:,:,3) * xzFactor;
xyz(:,:,2) = normalizedY .* xyz(:,:,3) * yzFactor;
end

