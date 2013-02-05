function [PredictedClasses] = classifyrt(T, Probabilities, data)

global cD cT cH cL cR cD2;

PredictedClasses = zeros(length(data),size(Probabilities,2));

for i = 1:length(data)
    n = 1;
    isleaf = 0;
    while (~isleaf)    % as long as its not a leaf
        if T(n, cL) == 0
            break
        end
        d = T(n, cD);
        d2 = T(n, cD2);
        t = T(n, cT);
        if (data(i, d) - data(i,d2)) > t
            n = T(n, cL);
        else
            n = T(n, cR);
        end
        isLeaf = T(n,cL) == 0;
    end
    %[~, class] = max(Probabilities(n,:));
    %PredictedClasses(i) = class;
    PredictedClasses(i,:) = Probabilities(n,:);
end