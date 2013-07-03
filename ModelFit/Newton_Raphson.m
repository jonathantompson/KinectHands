function [ root, x_guesses, y_guesses ] = Newton_Raphson( f, x, tollerance )
%NEWTON_RAPHSON Root finding using Newton Raphson method (without safeguards)
if(length(x) ~= 1)
    error('Newton_Raphson(): ERROR - x should be length 1');
end

% Check that func is a function handle
if(~isa(f,'function_handle'))
    error('Newton_Raphson(): ERROR - f should be a function handle');
end

% Get the derivative so we can evaluate it at certain points
symb = sym(f);  %% Create a symbolic expression from the input function
diff_symb = diff(symb);  %% Perform analytic differentiation
diff_handle = matlabFunction(diff_symb);  %% create a matlab handle so we can call the function

% Perform the Newton Raphson algorithm
x_guesses = zeros(1,100); % shouldn't take this many iterations, but pre-allocate for speed
y_guesses = zeros(1,100);
x_guesses(1) = x(1);
y_guesses(1) = f(x_guesses(1));
ind = 1;
max_iterations = 100;
while(ind <= max_iterations && abs(y_guesses(ind)) > tollerance)
    dy_dx = diff_handle(x_guesses(ind));  % Linearize around the current guess
    % Estimate the step size to make y = 0
    delta_x = -y_guesses(ind) / dy_dx;
    if (abs(delta_x) < tollerance)  % If the step size is less than machine tollerance
        break;
    end
    x_guesses(ind+1) = x_guesses(ind) + delta_x;
    ind = ind + 1;
    y_guesses(ind) = f(x_guesses(ind));
end

x_guesses = x_guesses(1:ind);
y_guesses = y_guesses(1:ind);

root = x_guesses(end);
