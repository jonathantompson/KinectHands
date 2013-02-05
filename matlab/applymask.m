%% convert rgb to tiny image

function I = applymask(im, M) 
%% apply mask and normalize hand image
    
    ndim1 = ndims(im);
    ndim2 = ndims(M);
        
    if (ndim1 == ndim2 == 2)
        I = applymasktoimage(im, M);
    elseif (ndim1 == 3 && ndim2 == 2)
        M = repmat(M, 1, size(im,3));
        I = applymasktoimage(im, M);
    elseif (ndim1 == 3 && ndim2 == 3)
        I = applymasktoimage(im, M);        
    elseif (ndim1 == 4 && ndim2 == 3)
        I = zeros(size(im));   
        N = size(im, 4);
        fprintf('Masking images...\n');
        for i = 1:N
            fprintf('Masking %g of %g.\n', i, N);
            M2 = repmat(M(:,:,i), 1, size(im, 3));
            I(:,:,:,i) = applymasktoimage(im(:,:,:,i), M2);
        end
    end
end

function I = applymasktoimage(im, M) 
    I = im;    
    I(M == 0) = 0;  
end