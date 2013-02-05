function [score, index] = match(probe, dbim)
    N = size(dbim, 4);
    
    probe = squeeze(probe);
    dbim = squeeze(dbim);
    
   
    if size(dbim, 3) == 4
        probe = repmat(probe, [1, 1, 1, N]);
        d = dbim - probe;
        d = d .* d;
        score = squeeze(sum(sum(sum(d))));
    else
        probe = repmat(probe, [1, 1, N]);
        d = dbim - probe;
        d = d .* d;
        score = squeeze(sum(sum(d)));
    end
    [score, index] = sort(score, 1, 'ascend');
    
    size(score);
end