function [RGB, MASK] = extractRenderedHandFromFile(pathtofile)

        RGB = imread(pathtofile);
        MASK = RGB(:,:,1) ~= 0;

end