function [ c, iteration_num ] = Levenberg_Marquardt( f, x, y, c_0, partial_derivs, delta_c_tollerence )
% Use Levenberg-Marquardt method to solve non-linear least squares such that we
% find the best c to fit the (x,y) data to the function y=f(x,c), where y
% is non-linearly dependant on the fitting coefficients c

% First check the inputs
% Check that func is a function handle
if(~isa(f,'function_handle'))
    error('Gauss_Newton(): ERROR - f should be a function handle');
end

% Check that the lenght of y and x agree
if(length(x) ~= length(y))
    error('Gauss_Newton(): ERROR - x and y vectors should be the same length');
end

if(length(c_0) ~= length(partial_derivs))
    error('Gauss_Newton(): ERROR - c_0 dimension is not equal to nvars');
end

if(~isa(partial_derivs,'cell'))
    error('Gauss_Newton(): ERROR - partial_derivs must be a cell array of function_handle');
end
for i = 1:length(partial_derivs)
    if(~isa(partial_derivs{i},'function_handle'))
        error('Gauss_Newton(): ERROR - partial_derivs must be a cell array of function_handle');
    end
end

% To calculate the jacobian matrix we need a closed form solution for the
% derivatives of func with respect to each c_i --> This is an input to this
% function above, so no need to calculate it here (it's up to the top user)

% Note: Matlab doesn't have a do-while loop construct.  This is horrible!
% Ohh Matalb!  Instead I'm escentially doing in my code:
% cond=false;
% while(cond) {
%    exp;
%    cond = test;
% }

iteration_num = 1;
c_k = c_0;
norm_delta_c = inf;
max_iterations = 100;
lambda_k = 0.01;
while norm_delta_c > delta_c_tollerence
    % 1. Evaluate the function at the x values using the current c_k estimate
    f_k = f(x,c_k);
    
    % compute the residual at iteration k.
    temp = f_k - y;
    r_k = temp'*temp;

    % Construct the Jacobian for this iteration
    J_k = zeros(length(x(:,1)),length(partial_derivs));
    for i_pderiv = 1:length(partial_derivs)
        % From wiki dr_i / dc_j = - Jij, where r_i = y_i - f(x_i,c_k)  --> Residual
        J_k(:,i_pderiv) = partial_derivs{i_pderiv}(x, c_k);  %% Evaluate the partial derivative with respect to c_i at the x values
    end
    
    % 2. Solve the linear system (5) as described in HW3 Q3.3
    delta_y = y-f_k;
    delta_y_prime = J_k' * delta_y;  % RHS
    normal_mat = J_k' * J_k;  % Symmetric real --> therefore we can use cholsky factorization
    normal_mat = normal_mat + lambda_k * diag(diag(normal_mat));
    [R,p] = chol(normal_mat);  % J_k = R'*R
    if (p > 0) 
        c = c_k;
        disp('Gauss_Newton: WARNING: R is not positive definite.  Cannot perform cholesky factorization');
        break;
    end
    % Solve for delta_c in normal_mat*delta_c = b_prime
    temp = R\delta_y_prime; % first triangular solve
    delta_c = R\(R'\temp); % second triangular solve

    % Compute the new residual using the updated_delta
    c_k_p1 = c_k + delta_c';
    f_k_p1 = f(x,c_k_p1);
    temp = f_k_p1 - y;
    r_k_p1 = temp'*temp;  %% FIRST RESIDUAL
    
    % 3. Repeat the linear solve for decreased lambda
    normal_mat = J_k' * J_k;  % Symmetric real --> therefore we can use cholsky factorization
    normal_mat = normal_mat + 0.5*lambda_k * diag(diag(normal_mat));
    [R,p] = chol(normal_mat);  % J_k = R'*R
    if (p > 0) 
        c = c_k;
        disp('Gauss_Newton: WARNING: R is not positive definite.  Cannot perform cholesky factorization');
        break;
    end
    % Solve for delta_c in normal_mat*delta_c = b_prime
    temp = R\delta_y_prime; % first triangular solve
    delta_c_prime = R\(R'\temp); % second triangular solve

    % Compute the new residual using the updated_delta
    c_k_p1_prime = c_k + delta_c_prime';
    f_k_p1_prime = f(x,c_k_p1_prime);
    temp = f_k_p1_prime - y;
    r_k_p1_prime = temp'*temp;  %% FIRST RESIDUAL
    
    % NOW WE HAVE 3 OPTIONS WITH 3 RESIDUALS
    force_iteration = 0;
    if (r_k < r_k_p1 && r_k < r_k_p1_prime)
        % Stepping increased the residual, double the dampening factor and
        % try again;
        lambda_k = lambda_k * 2;
        delta_c = delta_c * 0;
        force_iteration = 1;
    else if (r_k_p1 < r_k && r_k_p1 < r_k_p1_prime)
            lambda_k = lambda_k;  % Obviously not necessary, but for clarity
            delta_c = delta_c;
        else  %% (r_k_p1_prime < r_k && r_k_p1_prime < r_k_p1)
            lambda_k = lambda_k * 0.5;  % Obviously not necessary, but be obvious
            delta_c = delta_c_prime;
        end
    end
    
    % Now update with the best delta_c
    c_k = c_k + delta_c';
    
    if(force_iteration == 0)
        norm_delta_c = norm(delta_c);
        disp(['Iteration ',num2str(iteration_num),': 2-norm of delta_c is ',num2str(norm_delta_c)]);
    else
        norm_delta_c = delta_c_tollerence * 2;
    end
    iteration_num = iteration_num + 1;
    if(iteration_num > max_iterations)
        c = c_k;
        disp(['Levenberg_Marquardt: WARNING: hit max number of iterations (',num2str(max_iterations),')']);
        break;
    end
end

c = c_k;
