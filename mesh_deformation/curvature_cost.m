clear all; clc; close all;

angles = 0:0.01:pi;  %% 0 --> 180deg
for cur_angle = 1:length(angles)
    vec1(cur_angle,:) = [cos(angles(cur_angle)) sin(angles(cur_angle)) 0];
end
normr(vec1);  %% normalize the rows of vec1 (should already be unit)
vec2 = [1 0 0];
for cur_angle = 1:length(angles)
    dots(cur_angle) = dot(vec1(cur_angle,:),vec2);
end

cost = 1 ./ (dots + 1.05);
plot(360*angles ./ (2*pi), cost);
xlabel('normal angle');
ylabel('cost');
grid on;

% figure;
% cost = acos(dots);
% plot(360*angles ./ (2*pi), cost);
% xlabel('normal angle');
% ylabel('cost');
% grid on;