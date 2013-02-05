%clear
%load ~/Desktop/mnist/demo_data.mat
%[~, trainLabels] = max(trainLabels, [], 2);
%[~, testLabels] = max(testLabels, [], 2);


clear
fprintf('Generating fake data...\n');
fakedata

WL = ones(length(-15:.5:40), 2);
n = 1;
for i = -15:.5:40
    for d = 1:2
        WL(n, 1) = d;
        WL(n, 2) = i;
        n = n + 1;
    end
end

IX = randperm(length(WL));

global cD cT cH cL cR;

cD = 1;
cT = 2;
cH = 3;
cL = 4;
cR = 5;

[T1, I1, P1] = trainrt(data, labels, WL(IX(1:20),:), 10);
Pred1 = classifyrt(T1, P1, data);

[T2, I2, P2] = trainrt(data, labels, WL(IX(21:40),:), 10);
Pred2 = classifyrt(T2, P2, data);

[T3, I3, P3] = trainrt(data, labels, WL(IX(41:60),:), 10);
Pred3 = classifyrt(T3, P3, data);

predictedclasses = (Pred1 + Pred2 + Pred3) / 3;

[~, predictedclasses] = max(predictedclasses, [], 2);
%PredictedClasses(i) = class;

colormap('jet');
colors = 1:10;

figure(1);
hold on;
for ii = 1 : size(data, 1)
  scatter(data(ii,1), data(ii,2), 30, colors(predictedclasses(ii)), '+');
  %scatter(testdata(ii,1), testdata(ii,2), 30, colors{predictedclasses(ii)}, '+');
end
hold off;
