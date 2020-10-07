% 
% Copyright (c) 2020 Flight Dynamics and Control Lab
% 
% Permission is hereby granted, free of charge, to any person obtaining a
% copy of this software and associated documentation files (the 
% "Software"), to deal in the Software without restriction, including 
% without limitation the rights to use, copy, modify, merge, publish, 
% distribute, sublicense, and/or sell copies of the Software, and to permit
% persons to whom the Software is furnished to do so, subject to the 
% following conditions:
% 
% The above copyright notice and this permission notice shall be included
%  in all copies or substantial portions of the Software.
% 
% THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
% OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF 
% MERCHANTABILITY,FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
% IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY 
% CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
% TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
% SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
%%
close all;
addpath('aux_functions');
addpath('test_functions');
addpath('qp_traj_gen');

g = 9.81;
ad = [1, 1, -g];

zw = [0.0, 0.0, 1.0];
fd = ad + g * zw


yaw = 0.0;
r2c = [-sin(yaw),cos(yaw),0.0]'

d_r2c = [-sin(yaw),-sin(yaw),0.0]'

r3 = fd/norm(fd);
r1 = cross(r2c,r3)/norm(cross(r2c,r3));
r2 = cross(r3,r1);


r1
r2
r3
% X_n = [];
% for t=1:1:10
%     X_n(t)  = 10;

% end
% X_n