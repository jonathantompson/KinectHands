function I = applylcn(images)
    G = fspecial('gaussian',[3 3],1);
    if (iscell(images))
        I = images;
        for i = 1:length(images)
            I{i} = applylcnonce(I{i},G);
        end
    else
        I = applylcnonce(images, G);
    end
end

function I = applylcnonce(im, G)
    Ig = imfilter(im,G,'same');
    Ig = max(Ig, 0.04);
    I = im ./ Ig;
end