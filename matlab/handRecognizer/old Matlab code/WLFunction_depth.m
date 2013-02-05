function [ output ] = WLFunction( data, indices, WLCoeff )

% data:            is the input image --> as a 1D array
% uv:              is the uv index of the test pixel
% WLCoefficients:  are the current coefficients of this weak lerner
%                  WLCoeff(1) - threshold
% output:          binary output decision (0 = left subtree, 1 = right)

output = data(indices) < WLCoeff(1);

end

