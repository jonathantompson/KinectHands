function [DB] = extractrawhandsfromdirectory(pathtodirectory)
    
    % olddir = pwd();
    % cd(pathtodirectory);
    if (pathtodirectory(end) ~= '/')
        pathtodirectory = strcat(pathtodirectory,'/');
    end
    
    listing = dir(strcat(pathtodirectory,'hands_*.bin'));
    N = length(listing);
    
    % cd(olddir);
    
    height = 480;
    width = 640;

    DB = [];
    DB.XYZ = zeros(height, width, 3, N);
    DB.RGB = zeros(height, width, 3, N);
    DB.MASK = zeros(height, width, N);
    DB.DEPTH = zeros(height, width, N);
    for i = 1:N
        filename = [pathtodirectory, listing(i).name];
        fprintf('%g of %g: %s\n',i,N,filename);
        [xyz, rgb, depth, mask] = extractrawhandsfromfile(filename);
        DB.XYZ(:,:,:,i) = xyz;
        DB.RGB(:,:,:,i) = rgb;
        DB.DEPTH(:,:,i) = depth;
        DB.MASK(:,:,i) = mask;
    end
    
    fprintf('Creating masks\n');
    LHMASK = DB.MASK == 1;
    RHMASK = DB.MASK == 2;
    
    fprintf('Applying mask to rgb left\n');
    DB.RGB_LEFTONLY = applymask(DB.RGB, LHMASK);
    fprintf('Applying mask to rgb right\n');
    DB.RGB_RIGHTONLY = applymask(DB.RGB, RHMASK);
    fprintf('Applying mask to depth left\n');
    DB.DEPTH_LEFTONLY = applymask(DB.DEPTH, LHMASK);
    fprintf('Applying mask to depth right\n');
    DB.DEPTH_RIGHTONLY = applymask(DB.DEPTH, RHMASK);

    
    
end