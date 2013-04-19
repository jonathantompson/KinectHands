clearvars; close all; clc;

gamma = 250;
adjacency_beta = 0.2;
sink_source_beta = 5.0;

depths = logspace(-1, 3, 100);
E_a = gamma * exp(-depths.*depths * adjacency_beta);
% E_a = gamma * exp(depths.*depths * adjacency_beta);
semilogx(depths, E_a);
grid on;
xlabel('Delta Depth');

radius = 4;
num_samples = (radius*2+1)*(radius*2+1);
count = 0:1:num_samples;
D_k0 = count ./ num_samples;  %% Linear 0 to 1
D_k1 = 1 - D_k0;  %% Linear 1 to 0

% D_k0 = exp(-D_k0 * sink_source_beta)
% D_k1 = exp(-D_k1 * sink_source_beta)

D_k0 = exp(-D_k0 * sink_source_beta)
D_k1 = exp(-D_k1 * sink_source_beta)

% D_k0 = exp(D_k0.*D_k0 * sink_source_beta)-1
% D_k1 = exp(D_k1.*D_k1 * sink_source_beta)-1

figure;
subplot(2,1,1);
plot(count, D_k0);
subplot(2,1,2);
plot(count, D_k1);
xlabel('Number of integrated labels');