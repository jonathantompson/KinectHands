%% convert rgb to tiny image

function I = maketiny(im, newsize)
%% apply mask and normalize hand image
   
    if (nargin < 2)
        newsize = [32 32];
    end
    
    if (ndims(im) == 4)
        [h, w, c, N] = size(im);
        I = zeros(newsize(1), newsize(2), c, N);
        for i = 1:N
            fprintf('Making tiny (%g of %g)\n', i, N);
            theim = im(:,:,:,i);
            M = theim(:,:,1) ~= 0;
            I(:,:,:,i) = maketinyonce(theim, M, newsize);
        end
    elseif (ndims(im) == 3)
        if (size(im,3) == 3 || size(im, 3) == 4) % do single RGB/D
            theim = im;
            M = theim(:,:,1) ~= 0;
            I = maketinyonce(theim, M, newsize);
        else                 % do multiple monochrome
            [h, w, N] = size(im);
            I = zeros(newsize(1), newsize(2), N);
            for i = 1:size(im, 3)
               fprintf('Making tiny (%g of %g)\n', i, N);
                theim = im(:,:,i);
                M = theim(:,:,1) ~= 0;
                I(:,:,i) = maketinyonce(theim, M, newsize);               
            end
        end
    end    
end

function I = maketinyonce(im, M, newsize) 
%% apply mask and normalize hand image
        
    V = im;
    % apply mask
    V(~M) = 0;

    % find bounding rectangle
    [mrows, mcols] = find(M);
    minrow = min(mrows);
    maxrow = max(mrows);
    mincol = min(mcols);
    maxcol = max(mcols);
    bheight = 1 + maxrow - minrow;
    bwidth = 1 + maxcol - mincol;
    span = max([bheight,bwidth]);

    % paste rectangle into center of I
    I = zeros(span, span, size(im, 3));
    if (bheight == bwidth)
        I = V;
    elseif (bheight > bwidth)
        o = ceil((bheight - bwidth) / 2);
        I(:,o:o+bwidth-1, :)=V(minrow:maxrow,mincol:maxcol, :);
    else
        o = ceil((bwidth - bheight) / 2);
        I(o:o+bheight-1,:, :)=V(minrow:maxrow,mincol:maxcol, :);
    end
    
    % make tiny
    I = imresize(I, newsize,'method','nearest');

end