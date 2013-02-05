% test data on one random tree and associated leaf-class probabilities

function accuracy = evaluatert(data, labels, T, P)

pptest = classifyrt(T, P, data);
    
[~, predicted_labels] = max(pptest, [], 2);
    
accuracy = 100 * sum(labels == predicted_labels) / length(labels);
