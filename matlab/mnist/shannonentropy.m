function H = shannonentropy(X)
    P = histc(X(:),[1:10, Inf]);
    P = P / length(X(:));
    H0 = -P .* log2(P);
    H0(isnan(H0)) = 0;
    H = sum(H0);
end

