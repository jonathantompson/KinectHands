function [ ] = plotContour( xyz_data )

plot(xyz_data(:,1), xyz_data(:,2), 'Color', getColor(1)); hold on;
plot([xyz_data(end,1) xyz_data(1,1)], [xyz_data(end,2) xyz_data(1,2)], 'Color', getColor(1));
axis([-150 150 -150 150]);

end

