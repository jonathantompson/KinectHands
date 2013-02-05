A = tiny;
imshow(A);
A2 = edge(A, 'canny', [0.01 0.3], 1.0);
imshow(A2);