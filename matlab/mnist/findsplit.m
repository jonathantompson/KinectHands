function [x, u, I1, bestgain, bestH1, bestH2] = findsplit(D, L, I0, X, U, H0) 

s0 = sum(I0);
bestgain = -1;
for i = 1:length(X)
    for j = 1:length(U)
        M = I0 & (D(I0, X(i)) - D(I0, U(j)) > 0);   % did test pass threshold?
        H1 = shannonentropy(L(M));
        H2 = shannonentropy(L(~M));
        s1 = sum(M);
        s2 = s0 - s1;
        gain = H0 - (H1 * s1/s0 + H2 * s2/s0); 
        fprintf('Tried params %d, %d and got gain %f\n', X(i), U(j), gain);
        if (gain > bestgain)
            I1 = M;
            bestgain = gain;
            x = X(i);
            u = U(j);
            bestH1 = H1;
            bestH2 = H2;
        end
    end
end