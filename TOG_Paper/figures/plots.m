clearvars; clc; clear global; close all; format shortG;
addpath('./export_fig/');

filename = 'kinect_texture.bin';
width = 640;
height = 480;
channels = 1;
type = 'int16';
kinect = single(loadImageFile(filename, width, height, channels, type));

filename = 'synth_texture.bin';
type = 'single';
synthetic = single(loadImageFile(filename, width, height, channels, type));

filename = 'residue_texture.bin';
type = 'single';
residue = single(loadImageFile(filename, width, height, channels, type));

% Calculate the center of mass of the sythetic
com = [0,0];
min_max_depth = [inf, -inf];
num_pts = 0;
for v = 1:480
    for u = 1:640
        if (synthetic(v, u) ~= 0) 
            com = com + [u,v];
            num_pts = num_pts + 1;
            if (min_max_depth(1) > synthetic(v, u))
                min_max_depth(1) = synthetic(v, u);
            end
            if (min_max_depth(2) < synthetic(v, u))
                min_max_depth(2) = synthetic(v, u);
            end            
        end
    end
end
com = com / num_pts;

crop_size_horiz = 85;
crop_size_vert = 128;
bounds_u = [max(1, round(com(1)) - crop_size_horiz+10) min(640, round(com(1)) + crop_size_horiz)];
bounds_v = [max(1, round(com(2)) - crop_size_vert) min(480, round(com(2)) + crop_size_vert-10)];
kinect_cropped = kinect(bounds_v(1):bounds_v(2), bounds_u(1):bounds_u(2));
synthetic_cropped = synthetic(bounds_v(1):bounds_v(2), bounds_u(1):bounds_u(2));
residue_cropped = residue(bounds_v(1):bounds_v(2), bounds_u(1):bounds_u(2));

background = 2001;

synthetic_cropped(find(synthetic_cropped == 0)) = min_max_depth(2);
% residue_cropped(find(residue_cropped == 30)) = 0;

kinect_cropped = (kinect_cropped - min_max_depth(1))/ (min_max_depth(2)-min_max_depth(1));
synthetic_cropped = (synthetic_cropped - min_max_depth(1))/ (min_max_depth(2)-min_max_depth(1));

figure;
imshow(kinect_cropped, 'Border', 'tight');
set(gcf, 'Color', 'w');
export_fig PrimeSenseDepth.png -m2 -painters -a4 -nocrop
figure;
imshow(synthetic_cropped, 'Border', 'tight');
set(gcf, 'Color', 'w');
export_fig SyntheticDepth.png -m2 -painters -a4 -nocrop
figure;
imshow((residue_cropped - min(min(residue_cropped)))/ (0.75*(max(max(residue_cropped))-min(min(residue_cropped)))), 'Border', 'tight');
set(gcf, 'Color', 'w');
export_fig Residue.png -m2 -painters -a4 -nocrop

filename = 'rdf_depth.bin';
width = 640 / 4;
height = 480 / 4;
channels = 1;
type = 'int16';
kinect = single(loadImageFile(filename, width, height, channels, type));
min_max_depth = [inf, -inf];
for v = 1:height
    for u = 1:width
        if (kinect(v, u) < 1300) 
            if (min_max_depth(1) > kinect(v, u))
                min_max_depth(1) = kinect(v, u);
            end
            if (min_max_depth(2) < kinect(v, u))
                min_max_depth(2) = kinect(v, u);
            end            
        end
    end
end

im_kinect = (1-(kinect - min_max_depth(1))/ (min_max_depth(2)-min_max_depth(1)));
im_kinect(find(im_kinect<0)) = 0;
im_kinect = im_kinect * 0.7 + 0.3;

figure;
imshow(im_kinect, 'InitialMagnification', 400);


% figure;
% imshow((synthetic - min(min(synthetic)))/ (max(max(synthetic))-min(min(synthetic))));
% figure;
% imshow((residue - min(min(residue)))/ (max(max(residue))-min(min(residue))));