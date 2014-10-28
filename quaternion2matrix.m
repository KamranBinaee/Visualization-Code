function T = quaternion2matrix(Q)
    
    Q = Q/norm(Q); % Ensure Q has unit norm
    
    % Set up convenience variables
    w = Q(1); x = Q(2); y = Q(3); z = Q(4);
    w2 = w^2; x2 = x^2; y2 = y^2; z2 = z^2;
    xy = x*y; xz = x*z; yz = y*z;
    wx = w*x; wy = w*y; wz = w*z;
    
    T = [w2+x2-y2-z2 , 2*(xy - wz) , 2*(wy + xz) ,  0
         2*(wz + xy) , w2-x2+y2-z2 , 2*(yz - wx) ,  0
         2*(xz - wy) , 2*(wx + yz) , w2-x2-y2+z2 ,  0
              0      ,       0     ,       0     ,  1];
          
          
    % start matrix:
    %0.7609    0.6329    0.0017    0.1433
    
    % end matrix
    %0.9676    0.2396   -0.0180    0.0778
