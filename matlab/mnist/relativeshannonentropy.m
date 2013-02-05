function rH = relativeshannonentropy(X, Members)
    H0 = shannonentropy(X);
    H1 = shannonentropy(X(Members));
    H2 = shannonentropy(X(~Members));
    s0 = length(X(:));
    s1 = sum(Members);
    s2 = s0 - s1;
    rH = H0 - (H1 * s1/s0 + H2 * s2/s0); 
end