% GoHalfSitting.m
%%%%%%%%%%% Move to non singular pose %%%%%%%%%%%%
uLINK(RLEG_J2).q = -5.0*3.14/180;
uLINK(RLEG_J3).q = 10.0*3.14/180;
uLINK(RLEG_J4).q = -5.0*3.14/180;

uLINK(LLEG_J2).q = -5.0*3.14/180;
uLINK(LLEG_J3).q = 10.0*3.14/180;
uLINK(LLEG_J4).q = -5.0*3.14/180;


uLINK(BODY).p = [-0.2, -0.1, 0.7]';
uLINK(BODY).R = eye(3);
