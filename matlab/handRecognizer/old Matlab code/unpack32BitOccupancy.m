function [ occupancy_unpacked ] = unpack32BitOccupancy( occupancy_packed )
occupancy_unpacked = zeros(length(occupancy_packed)*32, 1, 'uint8');
for i = 1:32
    occupancy_unpacked(i:32:end) = ...
        squeeze(bitand(bitshift(occupancy_packed, -(i-1)), uint32(1)));
end

end

