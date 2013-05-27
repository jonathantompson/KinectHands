clearvars; clc; close all;

filename = 'heatmap_screenshot1.bin';
dim = 24;
dim_plot = 100;
nfeats = 8;
remove_outliers = false
outlier_threshold = 0.01
cur_feature = 5;

file = fopen(filename);
data = double(fread(file, dim * dim * nfeats, 'single'));
fclose(file);
hm = zeros(nfeats, dim, dim);
for i = 1:nfeats
    istart = (i-1)*(dim * dim) + 1;
    iend = (i)*(dim * dim);
    hm(i, :, :) = reshape(data(istart:iend), dim, dim);
end
cur_hm = abs(squeeze(hm(cur_feature,:,:)));
cur_hm = cur_hm / sum(sum(cur_hm));

%% Calculate the weighted mean
hm_mean = [0, 0];
sum_weights = 0;
for v = 1:dim
    for u = 1:dim
        if (remove_outliers)
            if (cur_hm(v, u) < 0.01)
                cur_hm(v, u) = 0;
            end
        end
        hm_mean = hm_mean + cur_hm(v, u) * [u, v];
        sum_weights = sum_weights + cur_hm(v, u);
    end
end
hm_mean = hm_mean / sum_weights;

%% Calculate the weighted std
hm_var = [0, 0];
non_zero_weights = 0;
for v = 1:dim
    for u = 1:dim
        if (cur_hm(v, u) > 1e-7)
            non_zero_weights = non_zero_weights + 1;
        end
        hm_var = hm_var + cur_hm(v, u) * ([u, v] - hm_mean).^2;
    end
end
hm_var = hm_var / (sum_weights * (non_zero_weights - 1) / non_zero_weights);
hm_std = sqrt(hm_var);

%% Alternatively use LM to do a non-linear least squares fit on the data
% gauss, X = [u, v], C = [A, meanU, meanV, stdU, stdV]
gauss = @(X, C) C(1) * exp(-((X(:,1)-C(2)).^2 ./ (2*C(4)) + (X(:,2)-C(3)).^2 ./ (2*C(5))));
X_vals = zeros(dim*dim, 2);
Y_vals = zeros(dim*dim, 1);
ind = 1;
for v = 1:dim
    for u = 1:dim
        X_vals(ind, 1) = u;
        X_vals(ind, 2) = v;
        Y_vals(ind, 1) = cur_hm(v, u);
        ind = ind + 1;
    end
end
% First we need to calculate all the partial derivatives of the function
% with respect to c1...
%%%% NOTE: Parital differentiation if c is represented by n variables, ie 
%%%% C1, C2, C3, C4.  Not C(1), ... , C(4) this is unfortunate, but diff 
%%%% doesn't recognize functions of vectors
func_no_vec = @(U,V,c1,c2,c3,c4,c5) c1 * exp(-((U-c2)^2 / (2*c4) + (V-c3)^2 / (2*c5)));
disp('df / dc1 = ');
disp(func2str(Partial_Deriv_From_Function_Handle(func_no_vec,1,'c1')));
disp('df / dc2 = ');
disp(func2str(Partial_Deriv_From_Function_Handle(func_no_vec,1,'c2')));
disp('df / dc3 = ');
disp(func2str(Partial_Deriv_From_Function_Handle(func_no_vec,1,'c3')));
disp('df / dc4 = ');
disp(func2str(Partial_Deriv_From_Function_Handle(func_no_vec,1,'c4')));
disp('df / dc5 = ');
disp(func2str(Partial_Deriv_From_Function_Handle(func_no_vec,1,'c5')));
% Now from the results above, hard code in the partial derivatives...  I
% hate having to do this, but matlab symbolic solver has trouble with
% functions of vectors :-(  Actually, there is a way... by parsing the text
% of c1 with c(1), c2 with c(2), etc and then creating a function handle
% form that...  But I think that's an overkill.
pderiv = cell(1,5);
pderiv{1} = @(x,c) exp(((x(:,1)-c(2)).^2.*(-1.0./2.0))./c(4)-((x(:,2)-c(3)).^2.*(1.0./2.0))./c(5));
pderiv{2} = @(x,c) (c(1).*exp(((x(:,1)-c(2)).^2.*(-1.0./2.0))./c(4)-((x(:,2)-c(3)).^2.*(1.0./2.0))./c(5)).*(x(:,1).*2.0-c(2).*2.0).*(1.0./2.0))./c(4);
pderiv{3} = @(x,c) (c(1).*exp(((x(:,1)-c(2)).^2.*(-1.0./2.0))./c(4)-((x(:,2)-c(3)).^2.*(1.0./2.0))./c(5)).*(x(:,2).*2.0-c(3).*2.0).*(1.0./2.0))./c(5);
pderiv{4} = @(x,c) c(1).*1.0./c(4).^2.*exp(((x(:,1)-c(2)).^2.*(-1.0./2.0))./c(4)-((x(:,2)-c(3)).^2.*(1.0./2.0))./c(5)).*(x(:,1)-c(2)).^2.*(1.0./2.0);
pderiv{5} = @(x,c) c(1).*1.0./c(5).^2.*exp(((x(:,1)-c(2)).^2.*(-1.0./2.0))./c(4)-((x(:,2)-c(3)).^2.*(1.0./2.0))./c(5)).*(x(:,2)-c(3)).^2.*(1.0./2.0);

c_0 = [max(max(cur_hm)), hm_mean(1), hm_mean(2), hm_std(1), hm_std(2)];
norm_delta_c_tollerence = 1e-5;
[c_solve] = Levenberg_Marquardt(gauss, X_vals, Y_vals, c_0, pderiv, norm_delta_c_tollerence);

%% Try BFGS
bfgs_func = @(C) sum(sum((Y_vals - gauss(X_vals, C)).^2));
h = 0.0001;
bfgs_J_func = {
    @(C) (bfgs_func(C+[h,0,0,0,0]')-bfgs_func(C-[h,0,0,0,0]'))/(2*h),...
    @(C) (bfgs_func(C+[0,h,0,0,0]')-bfgs_func(C-[0,h,0,0,0]'))/(2*h),...
    @(C) (bfgs_func(C+[0,0,h,0,0]')-bfgs_func(C-[0,0,h,0,0]'))/(2*h),...
    @(C) (bfgs_func(C+[0,0,0,h,0]')-bfgs_func(C-[0,0,0,h,0]'))/(2*h),...
    @(C) (bfgs_func(C+[0,0,0,0,h]')-bfgs_func(C-[0,0,0,0,h]'))/(2*h)};
c_solve_bfgs = BFGS_Backtracking(bfgs_func, bfgs_J_func, c_0', 1e-5, 1e-5, 1e-5, 1000);

%% Make some plots
fit_curve_lm = zeros(dim, dim);
fit_curve_lm_plot = zeros(dim_plot, dim_plot);
u_plot = zeros(dim_plot, dim_plot);
v_plot = zeros(dim_plot, dim_plot);
fit_curve_bfgs = zeros(dim, dim);
C = [1, hm_mean(1), hm_mean(2), hm_std(1), hm_std(2)];
for v = 1:dim
    for u = 1:dim
        fit_curve_lm(v, u) = gauss([u,v],c_solve);
        fit_curve_bfgs(v, u) = gauss([u,v],c_solve_bfgs);
    end
end
for v = 1:dim_plot
    for u = 1:dim_plot
        v_prime = v / (dim_plot / dim);
        u_prime = u / (dim_plot / dim);
        index = (v-1) * dim_plot + u;
        fit_curve_lm_plot(v, u) = gauss([u_prime,v_prime],c_solve);
        v_plot(v, u) = v_prime;
        u_plot(v, u) = u_prime;
    end
end
normalized_fit_curve_lm_plot = fit_curve_lm_plot / (sum(sum(fit_curve_lm)));
normalized_fit_curve_lm = fit_curve_lm / (sum(sum(fit_curve_lm)));
normalized_fit_curve_bfgs = fit_curve_bfgs / (sum(sum(fit_curve_bfgs)));
surf(u_plot, v_plot, normalized_fit_curve_lm_plot);
colormap('Default')
hold on;
surf(cur_hm)  %% Normalize by the sum of the weights
alpha(0.1)

figure; 
set(gcf,'Position',[200, 200, 1900, 500])
subplot(1,3,1);
pcolor(1:24, 1:24, normalized_fit_curve_lm);
colorbar;
title('Fit using LM');
subplot(1,3,2);
pcolor(1:24, 1:24, cur_hm);
colorbar;
subplot(1,3,3);
pcolor(1:24, 1:24, abs(cur_hm - normalized_fit_curve_lm));
colorbar;

figure; 
set(gcf,'Position',[200, 200, 1900, 500])
subplot(1,3,1);
pcolor(1:24, 1:24, normalized_fit_curve_bfgs);
colorbar;
title('Fit using BFGS');
subplot(1,3,2);
pcolor(1:24, 1:24, cur_hm);
colorbar;
subplot(1,3,3);
pcolor(1:24, 1:24, abs(cur_hm - normalized_fit_curve_bfgs));
colorbar;