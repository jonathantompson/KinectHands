% Try the unit simplex:
n = 30;
L = zeros(n,n);
vertices = zeros(n,n+1);
vertices(:,1) = zeros(n,1);  % Origin
vertices(:,2:end) = eye(n,n);
for i = 1:n
  L(:,i) = vertices(:,i+1) - vertices(:,1);
end
volume = det(L) / factorial(n)
expected_volume = 1 / (factorial(n))

X = -2:.1:2;
Y = -2:.1:2;
Z = zeros(length(Y), length(X));
for i = 1:length(Y)
  for j = 1:length(X)
    Z(i, j) = extendedRosenbrock([X(j) Y(i)]);
  end
end
surf(X, Y, Z);

[Val_1 I_1] = min(Z);  % Minimum element from each column
[Val_2 I_X] = min(Val_1);  % I_2 is the original column containing the minimum element
[Val_3 I_Y] = min(Z(:,I_X));
disp('Min value at: (x, y)');
disp(X(I_X));
disp(Y(I_Y));