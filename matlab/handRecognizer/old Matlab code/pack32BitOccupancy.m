function [ occupancy_packed ] = pack32BitOccupancy( occupancy_unpacked )

occupancy_packed = zeros(ceil(double(length(occupancy_unpacked))/32), 1, 'uint32');
for i = 1:32
    occupancy_packed = ...
        bitor(occupancy_packed, squeeze(bitshift(uint32(occupancy_unpacked(i:32:end)), (i-1))));
end

end

