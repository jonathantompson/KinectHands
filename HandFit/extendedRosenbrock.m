function [ f_x ] = extendedRosenbrock( x )

f_x = 0;
n = length(x);

if (mod(n, 2) ~= 0) 
  error('extendedRosenbrock: ERROR, x must be even!');
end

c = 100;
for i = 1:n-1
  f_x = f_x + (c * (x(i)*x(i) - x(i+1))*(x(i)*x(i) - x(i+1)) + (1 - x(i))*(1 - x(i)));
end

end