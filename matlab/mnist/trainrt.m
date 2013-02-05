% train a randomized tree

function [T, Probabilities, I] = trainrt(data, labels, WL, num_wls, max_depth)

global cD cT cH cL cR cD2;

total_wls = length(WL);

if (nargin < 4)
    num_wls = min(max(.001 * total_wls, 20),total_wls);
end

if (nargin < 5)
    max_depth = 20;
end

fprintf('Generating support structures\n');
num_categories = length(unique(labels));
T = zeros(1, 6);
I = {};
Probabilities = zeros(1, num_categories);

T(1,cH) = shannonentropy(labels);
%I{1} = logical(labels > 0 | labels <= 0);
I{1} = 1:length(labels);
GAIN_MIN = 0;
ENTROPY_CUTOFF = 0.01;
LEAF_SIZE = 5;
%LEAF_SIZE = 2;

fprintf('Beginning training...');
n = 1;
nodes = 1;
while (n <= nodes)
    bestgain = 0;
    h0 = T(n,cH);
    s0 = length(I{n});
    isleaf = (floor(log2(n)) >= max_depth - 1) || h0 <= ENTROPY_CUTOFF || length(I{n}) < LEAF_SIZE;
    if (~isleaf)
        wls = floor(rand(num_wls, 1) * total_wls) + 1;
        for i = 1:num_wls;
            I1 = I{n}((data(I{n}, WL(wls(i),1)) - data(I{n}, WL(wls(i),2))) > WL(wls(i),3));   % did test pass threshold?
            I2 = I{n}((data(I{n}, WL(wls(i),1)) - data(I{n}, WL(wls(i),2))) <= WL(wls(i),3));   % did test pass threshold?

            h1 = shannonentropy(labels(I1));
            h2 = shannonentropy(labels(I2));
            s1 = length(I1);
            s2 = length(I2);
            gain = h0 - (h1 * s1/s0 + h2 * s2/s0);
            if (gain > bestgain)
                bestgain = gain;
                bestp1 = WL(wls(i),1);
                bestp2 = WL(wls(i),2);
                bestp3 = WL(wls(i),3);
                besth1 = h1;
                besth2 = h2;
                bests1 = s1;
                bests2 = s2;
                bestI1 = I1;
                bestI2 = I2;
            end
        end
        if bestgain > GAIN_MIN
            T(n,cD) = bestp1;
            T(n,cD2) = bestp2;
            T(n,cT) = bestp3;
            fprintf('Node %d: found good weak learner...\n',n);
            fprintf('... with gain %f\n',bestgain);
            fprintf('... and parameters %d, %d, %f\n',T(n,cD),T(n,cD2),T(n,cT));
            fprintf('... my entropy is %f\n',T(n,cH));
            fprintf('... creating children');
            left = nodes + 1;
            right = nodes + 2;
            nodes = right;
            T(n,cL) = left;
            T(n,cR) = right;
            T(left,cH) = besth1;
            T(right,cH) = besth2;
            I{left} = bestI1;
            I{right} = bestI2;
            fprintf('... left branch gets %d labels and right get %d labels\n',bests1,bests2);
            fprintf('... my lt child (%d) will have entropy %f\n',left,T(left,cH));
            fprintf('... my rt child (%d) will have entropy %f\n',right,T(right,cH));
            if (T(n,cH) == T(left,cH) || T(n,cH) == T(right,cH))
                fprintf('... and for some reason one of my children is identical to me.\n');
            end
        else
            fprintf('Node %d: Not enough gain, becoming leaf node\n',n);
            isleaf = 1;
        end
    end
    if isleaf
        fprintf('Node %d: Made leaf node\n',n);
        Probabilities(n, :) = computeprobabilities(labels(I{n}), num_categories)';
    end
    n = n + 1;
end
fprintf('Finished constructing tree.  Tree has %d nodes\n',nodes);

end