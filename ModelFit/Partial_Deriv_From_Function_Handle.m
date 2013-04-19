function [ df_dvar_nthderiv ] = Partial_Deriv_From_Function_Handle( f, n, var )

% Check that func is a function handle
if(~isa(f,'function_handle'))
    error('Partial_Deriv_From_Function_Handle(): ERROR - f should be a function handle (ie @(x) 2x^2)');
end

% Get the derivative
symb = sym(f);  %% Create a symbolic expression from the input function
diff_symb = diff(symb, n, var);  %% Perform analytic differentiation
df_dvar_nthderiv = matlabFunction(diff_symb);  %% create a matlab handle so we can call the function

end

