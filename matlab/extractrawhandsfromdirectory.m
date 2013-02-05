function [DB] = extractrawhandsfromdirectory(pathtodirectory)
    
    tic 
    olddir = pwd();
    cd(pathtodirectory);
    if (pathtodirectory(end) ~= '/')
        pathtodirectory = strcat(pathtodirectory,'/');
    end
        
    FILES = rdir(strcat(pathtodirectory,'*/*.bin'));
    N = length(FILES);
    height = 480;
    width = 640;

    DB = [];
    fprintf('Creating memory for XYZ\n', N);
    DB.XYZ = zeros(height, width, 3, N);
    fprintf('Creating memory for RGBD\n', N);
    DB.RGBD = zeros(height, width, 4, N);
    fprintf('Creating memory for MASK\n', N);
    DB.MASK = zeros(height, width, N);
    fprintf('Creating memory for NAMES\n', N);
    DB.NAME = cell(1,1);

    for i = 1:N
        filename = FILES(i).name;
        fprintf('%g of %g: %s\n',i,N,filename);
        [xyz, rgbd, mask] = extractrawhandsfromfile(filename);
        DB.XYZ(:,:,:,i) = xyz;
        DB.RGBD(:,:,:,i) = rgbd;
        DB.MASK(:,:,i) = mask;
        DB.NAME{i} = relativepath(filename,pathtodirectory);
    end
    
    fprintf('Proactively saving work so far...\n');
    save('/Volumes/Extended/handcaptures/handcaptures.mat', 'DB', 'olddir', '-v7.3');
    clear all;
    fprintf('Reloading saved work so far...\n');
    load('/Volumes/Extended/handcaptures/handcaptures.mat');
    
    fprintf('Creating masks\n');
    LHMASK = DB.MASK == 1;
    RHMASK = DB.MASK == 2;
    
    fprintf('Applying mask to rgbd left\n');
    DB.RGBD_LEFT = applymask(DB.RGBD, LHMASK);
    fprintf('Applying mask to rgbd right\n');
    DB.RGBD_RIGHT = applymask(DB.RGBD, RHMASK);

    fprintf('Recentering RGBD_LEFT\n');
    DB.RGBD_LEFT = recenter(DB.RGBD_LEFT);
    fprintf('Recentering RGBD_RIGHT\n');
    DB.RGBD_RIGHT = recenter(DB.RGBD_RIGHT);

    fprintf('Making RGBD_LEFT_64')
    DB.RGBD_LEFT_64 = maketiny(DB.RGBD_LEFT, [64 64]);
    fprintf('Making RGBD_RIGHT_64')
    DB.RGBD_RIGHT_64 = maketiny(DB.RGBD_RIGHT, [64 64]);
    
    fprintf('Making RGBD_LEFT_32')
    DB.RGBD_LEFT_64 = maketiny(DB.RGBD_LEFT, [32 32]);
    fprintf('Making RGBD_RIGHT_32')
    DB.RGBD_RIGHT_64 = maketiny(DB.RGBD_RIGHT, [32 32]);
    
    cd(olddir);
    
    fprintf('Saving final data...\n');
    save('/Volumes/Extended/handcaptures/handcaptures.mat', 'DB', '-v7.3');

    toc
end
