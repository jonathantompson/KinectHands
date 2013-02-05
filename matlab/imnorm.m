function I = imnorm(images)
    if (iscell(images))
        I = images;
        for i = 1:length(images)
            I{i} = imnormonce(I{i});
        end
    else
        I = imnormonce(images);
    end
end

function I = imnormonce(im)
    m = mean(im(:));
    s = std(im(:));
    I = (im - m) / s;
end