function [ ] = plotShapeContext( shape_context )

cur_size = size(shape_context);
h = pcolor(1:cur_size(1),1:cur_size(2),shape_context); 
set(h, 'edgecolor','black');
% shading(gca,'interp'); 
colorbar;

end

