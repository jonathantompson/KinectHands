function [ str ] = modifiedcoeff2str( coeff )
  switch(coeff)
    case 1
      str = 'HAND POS X';
    case 2
      str = 'HAND POS Y';
    case 3
      str = 'HAND POS Z';
    case 4
      str = 'HAND ORIENT PHI';
    case 5
      str = 'HAND ORIENT THETA';
    case 6
      str = 'HAND ORIENT PSI';
    case 7
      str = 'WRIST THETA';
    case 8
      str = 'WRIST PHI';
    case 9
      str = 'THUMB THETA';
    case 10
      str = 'THUMB PHI';
    case 11
      str = 'THUMB K1 THETA';
    case 12
      str = 'THUMB K1 PHI';
    case 13
      str = 'THUMB K2 PHI';
    case 14
      str = 'F0 THETA';
    case 15
      str = 'F0 PHI';
    case 16
      str = 'F0 KNUCKLE CURL';
    case 17
      str = 'F1 THETA';
    case 18
      str = 'F1 PHI';
    case 19
      str = 'F1 KNUCKLE CURL';
    case 20
      str = 'F2 THETA';
    case 21
      str = 'F2 PHI';
    case 22
      str = 'F2 KNUCKLE CURL';
    case 23
      str = 'F3 THETA';
    case 24
      str = 'F3 PHI';
    case 25
      str = 'F3 KNUCKLE CURL';
    otherwise
      str = 'undefined';
  end
end

