function I = imsquare(im)
    if (size(im,1) == size(im,2))
        I = im;
    else
        rows = size(im,1);
        cols = size(im,2);
        colors = size(im,3);
        span = max(rows, cols);
        I = zeros(span, span, colors);
        for i = 1:colors
            I(1:rows,1:cols,i) = im(:,:,i);
        end
    end
end