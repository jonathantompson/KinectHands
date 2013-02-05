function [RGB, MASK] = extractRenderedHandsFromDirectory(pathtodirectory)
    
    olddir = pwd();
    cd(pathtodirectory);
    if (pathtodirectory(end) ~= '/')
        pathtodirectory = strcat(pathtodirectory,'/');
    end
    
    listing = dir(strcat(pathtodirectory,'*.png'));
    N = length(listing);
    RGB = cell(N,1);
    MASK = cell(N,1);
    fprintf('About to process %g files\n', N);

    for i = 1:N
        
        filename = listing(i).name;
        fprintf('(%g of %g): %s\n', i, N, filename);
        im = imread(filename);
        MASK{i} = im(:,:,1) ~= 0;
        RGB{i} = im;

    end
    cd(olddir);
    fprintf('Finished processing\n');
end