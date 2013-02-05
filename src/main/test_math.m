clear all; clc; close all;

format short e

A = [2*pi, 6912698, 1.56234524e-2]';  %% column vector

H(1,1) = 0;       H(1,2) = 0;       H(1,3) = 2;      
H(2,1) = 0;       H(2,2) = -1;      H(2,3) = 0;      
H(3,1) = 2;       H(3,2) = 0;       H(3,3) = 0;         

% Test case 1
% D(1,1) = 0.1;       D(1,2) = 1.7;       D(1,3) = 2.13;       E(1,1) = 3.19;       E(1,2) = 4.25;       E(1,3) = 5;
% D(2,1) = 6.2;       D(2,2) = 7.8;       D(2,3) = 8.14;       E(2,1) = 9.20;       E(2,2) = 10.26;      E(2,3) = 11;
% D(3,1) = 12.1609;   D(3,2) = 13.9;      D(3,3) = 14.15;      E(3,1) = 15.2557;    E(3,2) = 16.27;    E(3,3) = 17;      
% F(1,1) = 18.4;      F(1,2) = 19.1;      F(1,3) = 20.16;      G(1,1) = 21.22;      G(1,2) = 22.28;      G(1,3) = 23;   
% F(2,1) = 24.5;      F(2,2) = 25.11;     F(2,3) = 26.17;      G(2,1) = 27.23;      G(2,2) = 28.29;      G(2,3) = 29;   
% F(3,1) = 30.1871;   F(3,2) = 31.12;     F(3,3) = 32.18;      G(3,1) = 33.3229;    G(3,2) = 34.30;    G(3,3) = 35;  

% Test case 2
D(1,1) = 3.8499947236e+05;  D(1,2) = -1.3266829721e-02; D(1,3) = 1.2500010544e+05;  E(1,1) = 8.4445619490e-02;  E(1,2) = 2.1215441744e+04;  E(1,3) = 5.0999957781e+03;
D(2,1) = -1.3266829721e-02; D(2,2) = 1.2500010544e+05;  D(2,3) = 1.9971740626e-10;  E(2,1) = 2.1215441744e+04;  E(2,2) = 1.3892864032e-05;  E(2,3) = -1.3266829503e-04;
D(3,1) = 1.2500010544e+05;  D(3,2) = 1.9971740626e-10;  D(3,3) = 3.7500031675e+05;  E(3,1) = 1.3892863805e-05;  E(3,2) = 4.2441354523e+04;  E(3,3) = 5.0000042219e+03;
F(1,1) = 8.4445619490e-02;  F(1,2) = 2.1215441744e+04;  F(1,3) = 1.3892863805e-05;  G(1,1) = 5.0999957781e+03;  G(1,2) = -1.3266829503e-04; G(1,3) = 8.4459512359e-04;
F(2,1) = 2.1215441744e+04;  F(2,2) = 1.3892864032e-05;  F(2,3) = 4.2441354523e+04;  G(2,1) = -1.3266829503e-04; G(2,2) = 5.0000042219e+03;  G(2,3) = 6.3656796267e+02;
F(3,1) = 5.0999957781e+03;  F(3,2) = -1.3266829503e-04; F(3,3) = 5.0000042219e+03;  G(3,1) = 8.4459512359e-04;  G(3,2) = 6.3656796267e+02;  G(3,3) = 1.0100000000e+02;

%% Print them out just to be sure
A
H
D
E
F
G

G_inv = inv(G)
H_inv = inv(H)
disp(['det(G) = ',num2str(det(G))]);
disp(['det(H) = ',num2str(det(H))]);

temp1 = G_inv * F
temp2 = E * temp1
temp3 = D'
temp4 = G_inv + F
temp5 = G_inv - F

temp6 = F * A
temp7 = F' * A
temp8 = A' * D
temp9 = temp8 + temp7'
temp10 = temp8 - temp7'
temp11 = cross(temp8,temp7)
disp(['dot(temp8,temp7) = ',num2str(dot(temp8,temp7))]);

%% Try eigenvector and Eigenvalue calculation
temp12 = D * D';
[EigenVectors,eigenValues] = eig(temp12);  %% the eigen vectors are colums in the return matrix
eigenValues = [eigenValues(1,1) eigenValues(2,2) eigenValues(3,3)]'
eigenVector0 = [EigenVectors(1,1) EigenVectors(2,1) EigenVectors(3,1)]'
eigenVector1 = [EigenVectors(1,2) EigenVectors(2,2) EigenVectors(3,2)]'
eigenVector2 = [EigenVectors(1,3) EigenVectors(2,3) EigenVectors(3,3)]'

BIG_N = 10;
inA = zeros(BIG_N, BIG_N);
k1 = 1;
k2 = 1;
for i = 1:BIG_N
    for j = i:BIG_N
        inA(i,j) = k1;
        nextK = k1 + k2;
        k1 = k2;
        k2 = nextK;
        inA(j,i) = inA(i,j);
    end
end
inA
[EigenVectors,eigenValues] = eig(inA);  %% the eigen vectors are colums in the return matrix
for i = 1:length(inA)
    eigs(i) = eigenValues(i,i);
end
eigs