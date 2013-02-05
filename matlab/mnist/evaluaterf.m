% test forest of one or more trees on train and test datasets

function [Predicted_Labels, Overall, Individual] = evaluaterf(data, labels, Trees, Probabilities)

num_categories = length(unique(labels));
predicted_probabilities = zeros(length(labels),num_categories);
numtrees = length(Trees);
Individual = zeros(numtrees, 1);

for i = 1:numtrees
    if ~isempty(Trees{i})
        pp = classifyrt(Trees{i}, Probabilities{i}, data);
        predicted_probabilities = predicted_probabilities + pp;
        [~, Predicted_Labels] = max(pp, [], 2);
        Individual(i) = 100 * sum(labels == Predicted_Labels) / length(labels);    
    end
end

[~, Predicted_Labels] = max(predicted_probabilities, [], 2);
Overall = 100 * sum(labels == Predicted_Labels) / length(labels);
