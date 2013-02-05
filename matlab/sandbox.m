
%%

X = XYZ(:,:,1);
Y = XYZ(:,:,2);
Z = XYZ(:,:,3);
I = find(MASK == 1);
XYZ2 = [X(I), Y(I), Z(I)];
plot3(X(I), Y(I), Z(I),'o')

%%
%save('/Volumes/Extended/hands/fingerpoint.mat','DB','-v7.3');

%%
mydb = DB.RGBD_RIGHTONLY_64x64(:,:,4,:);
N = size(mydb, 4);
idx = randperm(N, 1)
I = mydb(:,:,:,idx);
% T = maketform('affine',[-0.5 0 0; 0.0 1.0 0; 0 0 1]);
% I2 = imtransform(I,T);
probe = I;
dbim = mydb;
[scores, indices] = match(probe, dbim);
bestmatches = dbim(:,:,:,indices);
peekdb(bestmatches(:,:,:,1:10),'all');
imshow(probe);

%%