function retval = perlin_noise_2d(x, y)
% Jonathan Tompson, Based on my C++ code implementation which is itself
% based Ken's version

  % A few nested helper functions
  function retfade = fade(t)
    retfade = t .* t .* t .* (t .* (t .* 6 - 15) + 10);
  end
  function retlerp = lerp(t, a, b)
    retlerp = a + t .* (b - a);
  end
  function retgrad = grad2d(hash, x, y)
    h = bitand(double(hash), 15);  %% CONVERT LO 4 BITS OF HASH CODE
    ugrad = x;
    ind = find(h >= 8);
    ugrad(ind) = y(ind);       %% INTO 12 GRADIENT DIRECTIONS
    vgrad = y;
    ind = find(h >= 4 * (h==12 + h==14));
    vgrad(ind) = x(ind);
    ind = find(h >= 4 * ((h==12 + h==14)==0));
    vgrad(ind) = zeros(1, length(ind));
    t1 = ugrad;
    ind = find(bitand(h,1) == 0);
    t1(ind) = -ugrad(ind);
    t2 = vgrad;
    ind = find(bitand(h,2) == 0);
    t2(ind) = -vgrad(ind);
    retgrad = t1+t2;
  end
%   function retgrad = grad2d(hash, x, y)
%     h = bitand(double(hash), 15);  %% CONVERT LO 4 BITS OF HASH CODE
%     if (h < 8)                     %% INTO 12 GRADIENT DIRECTIONS
%       ugrad = x;
%     else
%       ugrad = y;
%     end
%     if (h < 4)
%       vgrad = y;
%     else
%       if (h==12 || h==14)
%         vgrad = x;
%       else
%         vgrad = 0;
%       end
%     end
%     if (bitand(h,1) == 0)
%       retgrad = ugrad;
%     else
%       retgrad = -ugrad;
%     end
%     if (bitand(h,2) == 0)
%       retgrad = retgrad + vgrad;
%     else
%       retgrad = retgrad - vgrad;
%     end
%   end

persistent noise_permutation; 
if (isempty(noise_permutation))
  noise_permutation = [151,160,137,91,90,15,131,13,201,95,96,...
    53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,190,6,148,247,...
    120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,88,237,149,...
    56,87,174,20,125,136,171,168,68,175,74,165,71,134,139,48,27,166,77,146,...
    158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,102,...
    143,54,65,25,63,161,1,216,80,73,209,76,132,187,208,89,18,169,200,196,...
    135,130,116,188,159,86,164,100,109,198,173,186,3,64,52,217,226,250,124,...
    123,5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,...
    28,42,223,183,170,213,119,248,152,2,44,154,163,70,221,153,101,155,167,43,...
    172,9,129,22,39,253,19,98,108,110,79,113,224,232,178,185,112,104,218,246,...
    97,228,251,34,242,193,238,210,144,12,191,179,162,241,81,51,145,235,249,14,...
    239,107,49,192,214,31,181,199,106,157,184,84,204,176,115,121,50,45,127,4,...
    150,254,138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,...
    156,180,151,160,137,91,90,15,131,13,201,95,96,53,194,233,7,225,140,36,103,...
    30,69,142,8,99,37,240,21,10,23,190,6,148,247,120,234,75,0,26,197,62,94,...
    252,219,203,117,35,11,32,57,177,33,88,237,149,56,87,174,20,125,136,171,168,...
    68,175,74,165,71,134,139,48,27,166,77,146,158,231,83,111,229,122,60,211,...
    133,230,220,105,92,41,55,46,245,40,244,102,143,54,65,25,63,161,1,216,80,...
    73,209,76,132,187,208,89,18,169,200,196,135,130,116,188,159,86,164,100,...
    109,198,173,186,3,64,52,217,226,250,124,123,5,202,38,147,118,126,255,82,...
    85,212,207,206,59,227,47,16,58,17,182,189,28,42,223,183,170,213,119,248,...
    152,2,44,154,163,70,221,153,101,155,167,43,172,9,129,22,39,253,19,98,108,...
    110,79,113,224,232,178,185,112,104,218,246,97,228,251,34,242,193,238,210,...
    144,12,191,179,162,241,81,51,145,235,249,14,239,107,49,192,214,31,181,199,...
    106,157,184,84,204,176,115,121,50,45,127,4,150,254,138,236,205,93,222,114,...
    67,29,24,72,243,141,128,195,78,66,215,61,156,180];
end

X = bitand(floor(x), 255);  %% FIND UNIT CUBE THAT
Y = bitand(floor(y), 255);  %% CONTAINS POINT.
x = x - floor(x);  %% FIND RELATIVE X,Y,Z                     
y = y - floor(y);  %% OF POINT IN CUBE.
u = fade(x);  %% COMPUTE FADE CURVES
v = fade(y);  %% FOR EACH OF X,Y,Z.
 A = noise_permutation(X+1) + Y;   %% HASH COORDINATES OF
AA = noise_permutation(A+1); 
AB = noise_permutation(A+2);      
 B = noise_permutation(X+2) + Y;   %% THE 8 CUBE CORNERS,
BA = noise_permutation(B+1); 
BB = noise_permutation(B+2);      
        
retval = lerp(v, lerp(u, grad2d(noise_permutation(AA+1), x, y), ...     %% AND ADD
                         grad2d(noise_permutation(BA+1), x-1, y)), ...  %% BLENDED
                 lerp(u, grad2d(noise_permutation(AB+1), x, y-1), ...   %% RESULTS
                         grad2d(noise_permutation(BB+1), x-1, y-1)));   %% FROM 8 CORNERS OF CUBE
end