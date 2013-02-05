function [ str ] = coeff2str( coeff )
  switch(coeff)
    case 1
      str = 'HAND POS X';
    case 2
      str = 'HAND POS Y';
    case 3
      str = 'HAND POS Z';
    case 4
      str = 'HAND ORIENT X';
    case 5
      str = 'HAND ORIENT Y';
    case 6
      str = 'HAND ORIENT Z';
    case 7
      str = 'HAND ORIENT W';
    case 8
      str = 'WRIST THETA';
    case 9
      str = 'WRIST PHI';
    case 10
      str = 'THUMB THETA';
    case 11
      str = 'THUMB PHI';
    case 12
      str = 'THUMB K1 THETA';
    case 13
      str = 'THUMB K1 PHI';
    case 14
      str = 'THUMB K2 PHI';
    case 15
      str = 'F0 THETA';
    case 16
      str = 'F0 PHI';
    case 17
      str = 'F0 KNUCKLE CURL';
    case 18
      str = 'F1 THETA';
    case 19
      str = 'F1 PHI';
    case 20
      str = 'F1 KNUCKLE CURL';
    case 21
      str = 'F2 THETA';
    case 22
      str = 'F2 PHI';
    case 23
      str = 'F2 KNUCKLE CURL';
    case 24
      str = 'F3 THETA';
    case 25
      str = 'F3 PHI';
    case 26
      str = 'F3 KNUCKLE CURL';
    otherwise
      str = 'undefined';
  end
end

