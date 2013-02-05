clear all; close all; clear global; clc;

OPEN_FINGER_DOT_PROD_CUTOFF = -0.4;
phi_min = pi - acos(OPEN_FINGER_DOT_PROD_CUTOFF);
disp(['phi_min = ', num2str(360*phi_min/(2*pi)), 'deg']);
% march along the circle (on the sphere) from the max angle along the
% xaxis

% x^2 + y^2 + z^2 = 1 --> make y = 0
% x = sqrt(1-z^2)
% alternatively:
% x = r sin(phi) cos(theta)
% y = r sin(phi) sin(theta)
% z = r cos(phi)

phi = phi_min:0.01:pi;
theta = [zeros(1,length(phi)) pi*ones(1,length(phi))];
phi = [phi phi(end:-1:1)];
radius = 1;
x = radius .* sin(phi) .* cos(theta);
figure; plot(phi, x); xlabel('Phi'); ylabel('x');
z = radius .* cos(phi);
figure; plot(phi, z); xlabel('Phi'); ylabel('z');


% Stereographic projection
% X = x / (1-z)
% Y = y / (1-z)
X = x ./ (1-z);
max_rad_in_UV = 1 * sin(phi_min) / (1-cos(phi_min));
disp(['max_rad_in_UV = ', num2str(max_rad_in_UV)]);

X_rescaled = X / max_rad_in_UV;
figure; plot(X_rescaled); ylabel('bad scalling');

figure; plot(diff(X_rescaled)); ylabel('bad scalling diff');

X_different = sin(phi) ./ (1-cos(phi)) / max_rad_in_UV;
X_different(length(X_different)/2:end) = X_different(length(X_different)/2:end) * -1;
figure; plot(X_different); ylabel('bad scalling, different formulation');

rescale_factor = 2*atan(max_rad_in_UV);
x_rescaled_transformed = 2*atan(X) / rescale_factor;
figure; plot(x_rescaled_transformed); ylabel('improved scalling');


