function [ c, iteration_num ] = Gauss_Newton( f, x, y, c_0, partial_derivs, delta_c_tollerence )
% Use Gauss-Newton method to solve non-linear least squares such that we
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
max_iterations = 200;
while norm_delta_c > delta_c_tollerence
    % Evaluate the function at the x values using the current c_k estimate
    delta_y = y - f(x,c_k);
    
    % Construct the Jacobian for this iteration
    J_k = zeros(length(x),length(partial_derivs));
    for i_pderiv = 1:length(partial_derivs)
        % From wiki dr_i / dc_j = - Jij, where r_i = y_i - f(x_i,c_k)  --> Residual
        J_k(:,i_pderiv) = partial_derivs{i_pderiv}(x, c_k);  %% Evaluate the partial derivative with respect to c_i at the x values
    end
    
    % Now we need to solve the system:
    % J_k * delta_c = y - f(x,c_k) = delta_y  --> A * x = b
  
    % Next solve the linear system using the normal equations method (as 
    % suggested in the HW), where we instead solve:
    % (A'A)x = A'b 
%     normal_mat = J_k' * J_k;  % Symmetric real --> therefore we can use cholsky factorization
%     delta_y_prime = J_k' * delta_y;
%     [R,p] = chol(normal_mat);  % J_k = R'*R
%     if (p > 0) 
%         c = c_k;
%         disp('Gauss_Newton: WARNING: R is not positive definite.  Cannot perform cholesky factorization');
%         break;
%     end
%     % Solve for delta_c in normal_mat*delta_c = b_prime
%     temp = R'\delta_y_prime; % first triangular solve
%     delta_c = R\temp; % second triangular solve

    % SLOW METHOD:  Working on O(mxm), rather than O(n^2m)
    % The above method works (and was done so that I could get credit since 
    % it was asked in the HW), but full LU factorization is MUCH more robust
    % although it is a little slower.
    delta_c = J_k\delta_y;  
    
    % Now update c_k for the next iteration
    c_k = c_k + delta_c';
    
    norm_delta_c = norm(delta_c);
    disp(['Iteration ',num2str(iteration_num),': 2-norm of delta_c is ',num2str(norm_delta_c)]);
    iteration_num = iteration_num + 1;
    if(iteration_num > max_iterations)
        c = c_k;
        disp(['Gauss_Newton: WARNING: hit max number of iterations (',num2str(max_iterations),')']);
        break;
    end
end

c = c_k;
