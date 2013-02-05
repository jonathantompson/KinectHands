N = size(trainData,2);
X = randperm(1:N); X = X(1:10);
U = randperm(1:N); U = U(1:10);
D = 20;

for i:1