function [ setWLCoeffs ] = permuteWLCoeffs( WLCoeffs )
dim = length(WLCoeffs);
lengths = zeros(dim, 1);

num_WLPermutations = 1;
for n = 1:dim
    lengths(n) = length(WLCoeffs{n});
    num_WLPermutations = num_WLPermutations * lengths(n);
end

% Important, make the return type the same type as the first array (assume
% they're all of the same type).
setWLCoeffs = zeros(num_WLPermutations, dim, class(WLCoeffs{1}));

cur_indexes = ones(dim, 1);
cur_perm = 1;
while cur_perm <= num_WLPermutations
    for n = 1:dim
        setWLCoeffs(cur_perm, n) = WLCoeffs{n}(cur_indexes(n));
    end
    
    cur_indexes(1) = cur_indexes(1) + 1;
    for n = 1:dim
        if (cur_indexes(n) > lengths(n))
            cur_indexes(n) = 1;
            if (n < dim)
                cur_indexes(n + 1) = cur_indexes(n + 1) + 1;
            end
        else
            break;  % Only ripple up as far as we need to
        end
    end
            
    cur_perm = cur_perm + 1;
end
