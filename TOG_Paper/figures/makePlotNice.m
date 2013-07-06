function [ ] = makePlotNice( )
set(gca,'FontSize',18);
set(gca, 'FontName', 'Times New Roman'); 
set(get(gca, 'xlabel'), 'interpreter', 'latex'); 
set(get(gca, 'xlabel'), 'FontName', 'Times New Roman'); 
set(get(gca, 'xlabel'), 'FontSize', 18); 
set(get(gca, 'ylabel'), 'interpreter', 'latex'); 
set(get(gca, 'ylabel'), 'FontName', 'Times New Roman');
set(get(gca, 'ylabel'), 'FontSize', 18); 
set(legend(), 'interpreter', 'latex'); 
set(legend(), 'FontName', 'Times New Roman'); 
set(legend(), 'FontSize', 18); 
set(gcf, 'WindowStyle', 'normal'); 
set(gca, 'Unit', 'inches'); 
set(gca, 'Position', [.8 .6 4.5 3.125]); 
set(gcf, 'Unit', 'inches'); 
set(gcf, 'Position', [0.25 2.5 5.5 4.05]);
set(gcf, 'Color', 'w');
end

