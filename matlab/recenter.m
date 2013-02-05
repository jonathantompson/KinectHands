%% convert rgb to tiny image

function I = recenter(im)
%% apply mask and normalize hand image
   
    if (ndims(im) == 4)
        I = zeros(size(im));
        fprintf('Recentering multiple non-monochrome images\n');
        N = size(im, 4);
        for i = 1:N
            fprintf('recentering %g of %g\n', i, N);
            theim = im(:,:,:,i);
            M = theim(:,:,1) ~= 0;
            I(:,:,:,i) = recenterone(theim, M);
        end
    elseif (ndims(im) == 3)
        if (size(im,3) == 3 || size(im, 3) == 4) % do single RGB/D
            theim = im(:,:,1);
            M = theim ~= 0;
            I = recenterone(theim, M);
        else                 % do multiple monochrome
            I = zeros(size(im));
            fprintf('Recentering multiple monochrome images\n');
            N = size(im, 3);
            for i = 1:N
                fprintf('recentering %g of %g\n', i, N);
                theim = im(:,:,i);
                M = theim(:,:,1) ~= 0;
                I(:,:,i) = recenterone(theim, M);               
            end
        end
    end    
end

function I = recenterone(im, M)
    % find bounding rectangle
    [mrows, mcols] = find(M);
    minrow = min(mrows);
    maxrow = max(mrows);
    mincol = min(mcols);
    maxcol = max(mcols);
    bheight = 1 + maxrow - minrow;
    bwidth = 1 + maxcol - mincol;

    height = size(im, 1);
    width = size(im, 2);

    % paste rectangle into center of I
    I = zeros(size(im));

    x1 = max(ceil((width - bwidth) / 2), 1);
    x2 = min(x1 + bwidth - 1, width);
    y1 = max(ceil((height - bheight) / 2),1);
    y2 = min(y1 + bheight - 1, height);
    I(y1:y2,x1:x2, :)=im(minrow:maxrow,mincol:maxcol,:);
end