clear
load C:\Users\tompson\Documents\KinectHands\matlab\mnist/demo_data.mat
[~, trainLabels] = max(trainLabels, [], 2);
[~, testLabels] = max(testLabels, [], 2);

global cD cT cH cL cR cD2;

cD = 1;
cT = 2;
cH = 3;
cL = 4;
cR = 5;
cD2 = 6;

numtrees = 25;
Trees = cell(numtrees,1);
Probabilities = cell(numtrees,1);

pT = -1:(1/100):1.0;
%pT = [0.5];
pD = 1:size(testData,2);
pD2 = 1:size(testData,2);
WL = zeros(length(pT) * length(pD) * length(pD), 3);%
%WL = zeros(length(pD), 2);

n = 1;
for t = 1:length(pT)
    for d = 1:length(pD)
        for d2 = (d+1):length(pD)
            WL(n, 1) = d;
            WL(n, 2) = d2;
            WL(n, 3) = pT(t);
            %WL(n, 2) = 0.5;
            n = n + 1;
        end
    end
end

WL = WL(1:n-1, :);

test_results = zeros(1,numtrees);
train_results = zeros(1,numtrees);
figure;
for i = 1:numtrees

    % TRAIN
    [T, P] = trainrt(trainData, trainLabels, WL, 100, 8);
    Trees{i} = T;
    Probabilities{i} = P;

    % TEST
    [Predicted_Labels_Test, o_test, ind_test] = evaluaterf(testData, testLabels, Trees, Probabilities);
    fprintf('Overall accuracy for forest of %d trees on test data is %3.4f%%\n',i, o_test);
    [Predicted_Labels_Train, o_train, ind_train] = evaluaterf(trainData, trainLabels, Trees, Probabilities);
    fprintf('Overall accuracy for forest of %d trees on training data is %3.4f%%\n',i, o_train);
    fprintf('-------------------------------------------------------------\n');
    test_results(i) = o_test;
    train_results(i) = o_train;
    plot(1:i, test_results(1,1:i),'b-', 1:i, train_results(1,1:i), 'r-');
    grid on
    grid minor
    drawnow;
end

rp = randperm(length(testLabels));
tennumbers = testData(rp(1:10),:);
tenlabels = testLabels(rp(1:10));
figure;
for i = 1:10
    subplot(1,10,i);
    imshow(reshape(tennumbers(i,:),28,28)');
    title(['',num2str(Predicted_Labels_Test(rp(i)) - 1)]);
end