function [P] = computeprobabilities(X,number_of_categories)   
    P = histc(X(:),1:number_of_categories);
    P = P / length(X(:));
end