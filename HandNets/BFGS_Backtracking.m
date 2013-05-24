function [ x_star, f_x_star, H_x_star, x_vals, F_vals ] = ...
  BFGS_Backtracking( F, J, x0, tolx, toly, tolJ, maxiter )
% Check that func is a function handle
if (~isa(F, 'function_handle'))
  error('BFGS_Backtracking(): ERROR - F should be a function handle');
end
  
Jsize = size(J);
if (Jsize(1,:) ~= 1)
  error('BFGS_Backtracking(): ERROR - J should be a row vector');
end
for i = 1:length(J)
  if (~isa(J{i}, 'function_handle'))
    error(['BFGS_Backtracking(): ERROR - J{',num2str(i), ...
      '} should be a function handle']);
  end
end

% Perform the steepest descent algorithm
cur_x = x0;
cur_F = F(cur_x);
cur_B = eye(length(cur_x));  % Initial Hessian is the identity matrix
x_vals = zeros(length(x0), maxiter);  x_vals(:,1) = x0;
F_vals = zeros(1, maxiter); F_vals(1) = cur_F;
cur_J = zeros(length(J), 1);
for i = 1:length(J)
  cur_J(i, 1) = J{i}(cur_x);
end
n_s = 0.0001;
gamma = 0.5;
max_line_iterations = 10;

no_f_evals = 1;
no_J_evals = 1;
no_iterations = 0;
while(no_iterations <= maxiter)
  disp(['Iteration ', num2str(no_iterations)]);
  disp('   x_k^T = '); disp(num2str(cur_x'));
  disp(['   F_k = ', num2str(cur_F')]);
  disp(['   # Func evals = ', num2str(no_f_evals)]);
  disp(['   # Jacobian evals = ', num2str(no_J_evals)]);
  cur_B_str = num2str(cur_B);
  disp('   B_k = '); disp(cur_B_str);
  
  cur_J_norm = norm(cur_J, 2);
  disp(['   J_2norm = ', num2str(cur_J_norm)]);
  if (cur_J_norm < tolJ) 
    break;
  end
  
%   disp(['   J_k^T = ', num2str(cur_J')]);
  
  % Solve for the step size
  p_k = - (cur_B \ cur_J);
  
  % Now perform line search (binary) to find correct step size to ensure
  % strict decrease:
  alpha = 1;
  
  new_x = cur_x + alpha * p_k;
  new_F = F(new_x);
  no_f_evals = no_f_evals + 1;
  line_iterations = 0;
  
  while (new_F > (cur_F + n_s * alpha * cur_J' * p_k) && ...
      line_iterations < max_line_iterations)
    % Reduce alpha by the factor gamma
    alpha = alpha * gamma;
    new_x = cur_x + alpha * p_k;
    new_F = F(new_x);
    no_f_evals = no_f_evals + 1;
    line_iterations = line_iterations + 1;
  end
  
  if (line_iterations == max_line_iterations && ...
      new_F > (cur_F + n_s * alpha * cur_J' * p_k))
    disp('   Line search couldnt meet armijo conditions after 10 iterations.  Terminating...');
    break;
  end
  
  disp(['   alpha_k = ', num2str(alpha)]);

  % Take a step
  old_x = cur_x;
  cur_x = cur_x + alpha * p_k;
  no_iterations = no_iterations + 1;
  x_vals(:,no_iterations+1) = cur_x;
  old_F = cur_F;
  cur_F = F(cur_x);
  no_f_evals = no_f_evals + 1;
  F_vals(no_iterations+1) = cur_F;
  old_J = cur_J;
  for i = 1:length(J)
      cur_J(i, 1) = J{i}(cur_x);
  end
  no_J_evals = no_J_evals + 1;
  
  % Update the hessian approximation
  s = cur_x - old_x;
  y = cur_J - old_J;
  if (y'*s > 0)
    cur_B = cur_B - (cur_B*s*s'*cur_B) / (s' * cur_B * s) + (y*y')/(y'*s);
  else
    disp('   Skipping Hessian update...');
  end
  
  if (norm(old_x - cur_x, 2) < tolx)
    break;
  end
  if (abs(old_F - cur_F) < toly)
    break;
  end
end

x_star = cur_x;
f_x_star = F(x_star);
H_x_star = cur_B;

x_vals = x_vals(:,1:no_iterations);
F_vals = F_vals(1:no_iterations);
