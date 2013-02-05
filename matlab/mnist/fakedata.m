numClusters = 10;
ppc = 20; % points per cluster.

centers = rand(numClusters,2) * 30;


data = zeros(ppc*numClusters, 2);
labels = zeros(ppc*numClusters, 1);

pnt = 1;

for ii = 1 : numClusters
  data(pnt:pnt+ppc-1, :) = randn(ppc,2) + repmat(centers(ii,:), [ppc 1]);
  labels(pnt:pnt+ppc-1) = ii;
  pnt = pnt + ppc;
end

%colors = {'r', 'g', 'b', 'm'};
colormap('jet');
colors = 1:10;

figure(1);
clf;
hold on;
for ii = 1 : size(data, 1)
  %scatter(data(ii,1), data(ii,2), 30, colors{labels(ii)}, 'o');
  scatter(data(ii,1), data(ii,2), 30, colors(labels(ii)), 'o');
end
hold off;
