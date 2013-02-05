I = imread('/Volumes/Extended/hands/mayarenders/im.0001.png');
A = shiftdim(I, 2);
r = size(I, 1);
c = size(I, 2);
B = reshape(A, [3, r * c])';
colors = unique(B, 'rows')
fprintf('Found %g colors total\n', length(colors));
V = zeros(length(colors),1);
S = sortrows(B);
cnt = 1;
for i = 2:length(S)
    a = S(i,:);
    b = S(i-1,:);
    if (a(1) == b(1) && a(2) == b(2) && a(3) == b(3))
        V(cnt) = V(cnt) + 1;
    else
        cnt = cnt + 1;
    end
end
fprintf('Found %g colors with at least 100 pixels\n', sum(V > 100));
