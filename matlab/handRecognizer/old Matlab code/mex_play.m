clear all; close all; clear global; clc;

% On Mac OS X: follow these steps to get compiler to work:
% http://www.mathworks.com.au/support/solutions/en/data/1-FR6LXJ/

% On Windows, the quickest solution is to install the Microsoft Windows SDK
% if you don't want to instal the full blown VS2010.  VS2012 is not yet
% supported by matlab.  After the SDK install, run in the command window: 
% >> mex -setup

% Then make sure environment is setup correctly:
if (isempty(ls('timestwo.c')))
    copyfile([matlabroot '\extern\examples\refbook\timestwo.c'])
end
mex timestwo.c


mex -O arrayProduct.c
prod = arrayProduct(2, [4 356 213 4 5]);

mex -O matrixCopy.c
height = 3;
width = 4;
for v = 1:height
  for u = 1:width
    A(v,u) = (v-1)*width + u;
  end
end
A
B = matrixCopy(A);
B