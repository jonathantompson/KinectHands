function peekdb(im,type,r,c)
    if (length(size(im)) == 2)
        N = 1;
    else
        N = size(im,ndims(im));
    end
    
    if (nargin < 2)
        type = 'rand'
    elseif (nargin < 3)
        r = 5;
        c = 5;
    elseif (nargin < 4)
        c = 5;
        r = ceil(r / 5);
    end

    if (strcmp(type, 'all') == 1)
        r = floor(sqrt(N));
        c = ceil(N / r);
        idx = 1:N;
    elseif (strcmp(type, 'rand') == 1)
        idx = randperm(N, r*c);
    end
    
    figure;
    for i = 1:length(idx)
        subplot(r, c, i);
        if iscell(im)
            imshow(im{idx(i)});
        else
            if (ndims(im) == 3)
                imshow(im(:,:,idx(i)));
            else
                if (size(im,3) >= 3)
                    imshow(im(:,:,1:3,idx(i)));
                else
                    imshow(im(:,:,:,idx(i)));
                end
                    
            end
        end
        title(sprintf('%g',idx(i)));
    end
end