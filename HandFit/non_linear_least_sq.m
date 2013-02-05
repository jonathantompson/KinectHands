% Jonathan Tompson - 8/20
% This is from my solution to HW3 of Scientific Computing (spring 2012)

clear all; close all; clear global;
format long

fprintf('2. Nonlinear Least-Squares Fitting\n3.1\n');
FS = 16; % Font Size
x_interval = [0 10];
n_rand = 100; % number of random points
n_func = 1000; % number of function points (for just plotting the function)
e_preturb = 1e-2; % magnitude of random preturbations
c_correct = [1 0.5 2 0]; % correct solution
rng('default'); % Re-seed the pseudo-random number generator
% rand => uniformly distributed between 0 and 1 => So we need to re-scale
func = @(x,c) c(1)*exp(-c(2)*x) .* sin(c(3)*x+c(4));
x_vals_rand = x_interval(1) + (x_interval(2)-x_interval(1)) * rand(n_rand,1);
x_vals_func = linspace(x_interval(1),x_interval(2),n_func);
noise = e_preturb * randn(length(x_vals_rand),1); % with mean = 0 and s.t.d = e_preturb
y_vals_rand = func(x_vals_rand,c_correct) + noise; 
y_vals_func = func(x_vals_func,c_correct);

% Plot the random values and the function evals
figure;
set(gcf, 'color', 'white');
set(gcf, 'InvertHardCopy', 'off');
plot(x_vals_func,y_vals_func, 'r','LineWidth',1.5); hold on; grid on;
plot(x_vals_rand,y_vals_rand, 'ob');
set(gca,'FontSize',FS);
xlabel('X value','FontSize',FS);
ylabel('Y value','FontSize',FS);
legH = legend({'$f\left(x,c\right)=c_1e^{-c_2x}sin\left(c_3x+c_4\right)$     , where $c=\left[1,\frac{1}{2},2,0\right]$',...
    [num2str(n_rand),' values with gaussian noise $\left(\sigma=',num2str(e_preturb),'\right)$']},...
    'Location','NorthEast','FontSize',FS-5,'Interpreter','Latex');
% set(legH,'units','pixels');
% lp=get(legH,'outerposition');
% set(legH,'outerposition',[lp(1)*0.75,lp(2),lp(3)*1.1,lp(4)]);
fprintf('See plot for results\n\n');

%3.2
fprintf('\n3.2\n');
% First we need to calculate all the partial derivatives of the function
% with respect to c1...
%%%% NOTE: Parital differentiation if c is represented by n variables, ie 
%%%% C1, C2, C3, C4.  Not C(1), ... , C(4) this is unfortunate, but diff 
%%%% doesn't recognize functions of vectors
func_no_vec = @(x,c1,c2,c3,c4) c1*exp(-1*c2*x) .* sin(c3*x+c4);
disp('df / dc1 = ');
disp(func2str(Partial_Deriv_From_Function_Handle(func_no_vec,1,'c1')));
disp('df / dc2 = ');
disp(func2str(Partial_Deriv_From_Function_Handle(func_no_vec,1,'c2')));
disp('df / dc3 = ');
disp(func2str(Partial_Deriv_From_Function_Handle(func_no_vec,1,'c3')));
disp('df / dc4 = ');
disp(func2str(Partial_Deriv_From_Function_Handle(func_no_vec,1,'c4')));
% Now from the results above, hard code in the partial derivatives...  I
% hate having to do this, but matlab symbolic solver has trouble with
% functions of vectors :-(  Actually, there is a way... by parsing the text
% of c1 with c(1), c2 with c(2), etc and then creating a function handle
% form that...  But I think that's an overkill.
pderiv = cell(1,4);
pderiv{1} = @(x,c) exp(-c(2).*x).*sin(c(4)+c(3).*x);
pderiv{2} = @(x,c) -c(1).*x.*exp(-c(2).*x).*sin(c(4)+c(3).*x);
pderiv{3} = @(x,c) c(1).*x.*cos(c(4)+c(3).*x).*exp(-c(2).*x);
pderiv{4} = @(x,c) c(1).*cos(c(4)+c(3).*x).*exp(-c(2).*x);

% Now use Gauss Newton to solve the non-linear least squares fitting
c_0 = c_correct;  % First try a guess that's close to correct
c_0 = c_0 + 0.1;
norm_delta_c_tollerence = 1e-9;
[c_solve] = Gauss_Newton(func, x_vals_rand, y_vals_rand, c_0, pderiv, norm_delta_c_tollerence);
c_0_1 = [1 1 1 1];
[c_solve2] = Gauss_Newton(func, x_vals_rand, y_vals_rand, c_0_1, pderiv, norm_delta_c_tollerence);
% plot the result against the real curve
x = linspace(x_interval(1),x_interval(2),50);
y_vals_func_fit = func(x,c_solve);
y_vals_func_fit2 = func(x,c_solve2);
figure;
set(gcf, 'Position',[200 200 800 600]);
set(gcf, 'color', 'white');
set(gcf, 'InvertHardCopy', 'off');
plot(x_vals_func,y_vals_func, 'r','LineWidth',1.5); hold on; grid on;
plot(x,y_vals_func_fit, 'bo','LineWidth',1.5);
plot(x,y_vals_func_fit2, 'ko','LineWidth',1.5);
set(gca,'FontSize',FS);
xlabel('X value','FontSize',FS);
ylabel('Y value','FontSize',FS);
legH = legend({'$f\left(x,c\right)=c_1e^{-c_2x}sin\left(c_3x+c_4\right)$     , where $c=\left[1,\frac{1}{2},2,0\right]$',...
    [sprintf('$c_0=\\left<%.2f, %.2f, %.2f, %.2f\\right>$',c_0(1),c_0(2),c_0(3),c_0(4)),char(10),sprintf('      $\\Rightarrow$ $c_{fit}=\\left<%.3f, %.3f, %.3f, %.3f\\right>$',c_solve(1),c_solve(2),c_solve(3),c_solve(4))],...
    [sprintf('$c_0=\\left<%.2f, %.2f, %.2f, %.2f\\right>$',c_0_1(1),c_0_1(2),c_0_1(3),c_0_1(4)),char(10),sprintf('      $\\Rightarrow$ $c_{fit}=\\left<%.1e, %.1e, %.1e, %.1e\\right>$',c_solve2(1),c_solve2(2),c_solve2(3),c_solve2(4))]},...
    'Location','NorthEast','FontSize',FS-3,'Interpreter','Latex');
% set(legH,'units','pixels');
% lp=get(legH,'outerposition');
% set(legH,'outerposition',[lp(1)*0.75,lp(2),lp(3)*1.22,lp(4)]);
fprintf('See plot for results\n\n');
spread = max(y_vals_func)-min(y_vals_func);
axis([min(x) max(x) (min(y_vals_func)-0.1*spread) (max(y_vals_func)+0.1*spread)]);

disp(['norm-2(c-c_solve) = ',sprintf('%.4e',norm(abs(c_correct-c_solve),2)),' for c_0 = <',...
    num2str(c_0(1)),', ',num2str(c_0(1)),', ',num2str(c_0(1)),', ',num2str(c_0(1)),'>']);
disp(['norm-2(c-c_solve) = ',sprintf('%.4e',norm(abs(c_correct-c_solve2),2)),' for c_0 = <',...
    num2str(c_0_1(1)),', ',num2str(c_0_1(1)),', ',num2str(c_0_1(1)),', ',num2str(c_0_1(1)),'>']);


%3.3
fprintf('\n3.3\n');
% Now use Levenberg-Marquardt to solve the non-linear least squares fitting
c_0 = c_correct;  % First try a guess that's close to correct
c_0 = c_0 + 0.01;
norm_delta_c_tollerence = 1e-9;
[c_solve] = Levenberg_Marquardt(func, x_vals_rand, y_vals_rand, c_0, pderiv, norm_delta_c_tollerence);
c_0_1 = [1 1 1 1];
[c_solve2] = Levenberg_Marquardt(func, x_vals_rand, y_vals_rand, c_0_1, pderiv, norm_delta_c_tollerence);
c_solve2
% plot the result against the real curve
x = linspace(x_interval(1),x_interval(2),50);
y_vals_func_fit = func(x,c_solve);
y_vals_func_fit2 = func(x,c_solve2);
figure;
set(gcf, 'Position',[200 200 800 600]);
set(gcf, 'color', 'white');
set(gcf, 'InvertHardCopy', 'off');
plot(x_vals_func,y_vals_func, 'r','LineWidth',1.5); hold on; grid on;
plot(x,y_vals_func_fit, 'bo','LineWidth',1.5);
plot(x,y_vals_func_fit2, 'ko','LineWidth',1.5);
set(gca,'FontSize',FS);
xlabel('X value','FontSize',FS);
ylabel('Y value','FontSize',FS);
legH = legend({'$f\left(x,c\right)=c_1e^{-c_2x}sin\left(c_3x+c_4\right)$     , where $c=\left[1,\frac{1}{2},2,0\right]$',...
    [sprintf('$c_0=\\left<%.2f, %.2f, %.2f, %.2f\\right>$',c_0(1),c_0(2),c_0(3),c_0(4)),char(10),sprintf('      $\\Rightarrow$ $c_{fit}=\\left<%.3f, %.3f, %.3f, %.3f\\right>$',c_solve(1),c_solve(2),c_solve(3),c_solve(4))],...
    [sprintf('$c_0=\\left<%.2f, %.2f, %.2f, %.2f\\right>$',c_0_1(1),c_0_1(2),c_0_1(3),c_0_1(4)),char(10),sprintf('      $\\Rightarrow$ $c_{fit}=\\left<%.1e, %.1e, %.1e, %.1e\\right>$',c_solve2(1),c_solve2(2),c_solve2(3),c_solve2(4))]},...
    'Location','NorthEast','FontSize',FS-3,'Interpreter','Latex');
% set(legH,'units','pixels');
% lp=get(legH,'outerposition');
% set(legH,'outerposition',[lp(1)*0.75,lp(2),lp(3)*1.22,lp(4)]);
fprintf('See plot for results\n\n');
spread = max(y_vals_func)-min(y_vals_func);
axis([min(x) max(x) (min(y_vals_func)-0.1*spread) (max(y_vals_func)+0.1*spread)]);

disp(['norm-2(c-c_solve) = ',sprintf('%.4e',norm(abs(c_correct-c_solve),2)),' for c_0 = <',...
    num2str(c_0(1)),', ',num2str(c_0(2)),', ',num2str(c_0(3)),', ',num2str(c_0(4)),'>']);
disp(['norm-2(c-c_solve) = ',sprintf('%.4e',norm(abs(c_correct-c_solve2),2)),' for c_0 = <',...
    num2str(c_0_1(1)),', ',num2str(c_0_1(2)),', ',num2str(c_0_1(3)),', ',num2str(c_0_1(4)),'>']);

% Check the number of iterations each method takes to converge:
c_0 = [5 0.51 2.1 0.1];
[c_solve_LM, n_iterations_LM] = Levenberg_Marquardt(func, x_vals_rand, y_vals_rand, c_0, pderiv, norm_delta_c_tollerence);
[c_solve_GN, n_iterations_GN] = Gauss_Newton(func, x_vals_rand, y_vals_rand, c_0, pderiv, norm_delta_c_tollerence);
disp('Using Levenberg-Marquardt:');
disp(['norm-2(c-c_solve) = ',sprintf('%.4e',norm(abs(c_correct-c_solve_LM),2)),' for c_0 = <',...
    num2str(c_0(1)),', ',num2str(c_0(2)),', ',num2str(c_0(3)),', ',num2str(c_0(4)),'>']);
disp(['The number of iterations required to converge is = ',num2str(n_iterations_LM)]);
disp('Using Gauss-Newton:');
disp(['norm-2(c-c_solve) = ',sprintf('%.4e',norm(abs(c_correct-c_solve_GN),2)),' for c_0 = <',...
    num2str(c_0(1)),', ',num2str(c_0(2)),', ',num2str(c_0(3)),', ',num2str(c_0(4)),'>']);
disp(['The number of iterations required to converge is = ',num2str(n_iterations_GN)]);

% See how many bits accuracy we get when there is no input perturbation:
e_preturb = 0; % magnitude of random preturbations
norm_delta_c_tollerence = 1e-15;
y_vals_rand = func(x_vals_rand,c_correct); 
c_0_1 = c_correct;
[c_solve_LM1] = Levenberg_Marquardt(func, x_vals_rand, y_vals_rand, c_0_1, pderiv, norm_delta_c_tollerence);
c_0_2 = [1 1 1 1];
[c_solve_LM2] = Levenberg_Marquardt(func, x_vals_rand, y_vals_rand, c_0_2, pderiv, norm_delta_c_tollerence);
disp(['norm-2(c-c_solve) = ',sprintf('%.4e',norm(abs(c_correct-c_solve_LM1),2)),' for c_0 = <',...
    num2str(c_0_1(1)),', ',num2str(c_0_1(2)),', ',num2str(c_0_1(3)),', ',num2str(c_0_1(4)),'>']);
disp(['norm-2(c-c_solve) = ',sprintf('%.4e',norm(abs(c_correct-c_solve_LM2),2)),' for c_0 = <',...
    num2str(c_0_2(1)),', ',num2str(c_0_2(2)),', ',num2str(c_0_2(3)),', ',num2str(c_0_2(4)),'>']);
