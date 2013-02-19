function [ str ] = modifiedcoeff2str( coeff )
  switch(coeff)
    case 1
      str = 'HAND POS U';
    case 2
      str = 'HAND POS V';
    case 3
      str = 'HAND ORIENT X COS';
    case 4
      str = 'HAND ORIENT X SIN'; 
    case 5
      str = 'HAND ORIENT Y COS';
    case 6
      str = 'HAND ORIENT Y SIN'; 
    case 7
      str = 'HAND ORIENT Z COS';
    case 8
      str = 'HAND ORIENT Z SIN'; 
    case 9
      str = 'WRIST THETA COS';
    case 10
      str = 'WRIST THETA SIN';      
    case 11
      str = 'WRIST PHI COS';
    case 12
      str = 'WRIST PHI SIN';      
    case 13
      str = 'THUMB K1 U';
    case 14
      str = 'THUMB K1 V';      
    case 15
      str = 'THUMB K2 U';
    case 16
      str = 'THUMB K2 V';  
    case 17
      str = 'THUMB TIP U';
    case 18
      str = 'THUMB TIP V';  
    case 19
      str = 'F0 K1 U';
    case 20
      str = 'F0 K1 V';      
    case 21
      str = 'F0 K2 U';
    case 22
      str = 'F0 K2 V';         
    case 23
      str = 'F0 TIP U';
    case 24
      str = 'F0 TIP V';
    case 25
      str = 'F1 K1 U';
    case 26
      str = 'F1 K1 V';      
    case 27
      str = 'F1 K2 U';
    case 28
      str = 'F1 K2 V';       
    case 29
      str = 'F1 TIP U';
    case 30
      str = 'F1 TIP V';
    case 31
      str = 'F2 K1 U';
    case 32
      str = 'F2 K1 V';      
    case 33
      str = 'F2 K2 U';
    case 34
      str = 'F2 K2 V';       
    case 35
      str = 'F2 TIP U';
    case 36
      str = 'F2 TIP V';
    case 37
      str = 'F3 K1 U';
    case 38
      str = 'F3 K1 V';      
    case 39
      str = 'F3 K2 U';
    case 40
      str = 'F3 K2 V';           
    case 41
      str = 'F3 TIP U';
    case 42
      str = 'F3 TIP V';      
    otherwise
      str = 'undefined';
  end
end

