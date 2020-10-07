function desired = command_line(t)

Pxi = [-22.0000,76.5000, -91.0000,  37.5000,      0,    0,        0,     0]; %0->1
% Pxi = [18.0000, -63.5000, 77.0000, -32.5000, 0, 0, 0, 11.0000]; %11->10
% Pxi = [-22.0000, 76.5000,-91.0000, 37.5000,       0     ,  0      , 0,9.0000]; %9->10
Vxi = polyder(Pxi);
Axi = polyder(Vxi);
Jxi = polyder(Axi);
Sxi = polyder(Jxi);

Pyi = [-20.0000,   70.0000,  -84.0000,   35.0000,         0,     0,       0,     0]; %0->1
% Pyi = [20.0000,  -70.0000,   84.0000,  -35.0000,         0,         0,         0,   11.0000]; %11->10
% Pyi = [-20.0000, 70.0000,-84.0000, 35.0000,       0     ,  0    ,   0  ,9.0000]; %9->10
Vyi = polyder(Pyi);
Ayi = polyder(Vyi);
Jyi = polyder(Ayi);
Syi = polyder(Jyi);

Pzi = [-2.1880,   12.0110, -21.3580,  12.4850,    0,  0.0500,      0,       0]; %0->1
% Pzi = [37.8120, -127.9890,  146.6420,  -57.5150,         0,    0.0500,         0,   11.0000]; %11->10
% Pzi = [-2.1880, 12.0110,-21.3580, 12.4850   ,    0  ,0.0500     ,  0 , 9.0000]; %9->10
Vzi = polyder(Pzi);
Azi = polyder(Vzi);
Jzi = polyder(Azi);
Szi = polyder(Jzi);

X_n  = polyval(Pxi,t);
Xv_n  = polyval(Vxi,t);
Xa_n  = polyval(Axi,t);
Xj_n  = polyval(Jxi,t);
Xs_n  = polyval(Sxi,t);


Y_n  = polyval(Pyi,t);
Yv_n  = polyval(Vyi,t);
Ya_n  = polyval(Ayi,t);
Yj_n  = polyval(Jyi,t);
Ys_n  = polyval(Syi,t);

Z_n  = polyval(Pzi,t);
Zv_n  = polyval(Vzi,t);
Za_n  = polyval(Azi,t);
Zj_n  = polyval(Jzi,t);
Zs_n  = polyval(Szi,t);

desired.x = [X_n, Y_n, Z_n]';
desired.v = [Xv_n , Yv_n, Zv_n]';
desired.x_2dot = [Xa_n, Ya_n, Za_n]';
desired.x_3dot = [Xj_n, Yj_n, Zj_n]';
desired.x_4dot = [Xs_n, Ys_n, Zs_n]';

% using fixed yaw angle

% desired.b1 = [0, 0, 0]';
% desired.b1_dot =  desired.b1;
% desired.b1_2dot = desired.b1;
% w = 2 * pi / 10;
desired.b1 = [1, 0, 0]';
desired.b1_dot = [0, 0, 0]';
desired.b1_2dot = [0, 0, 0]';

end