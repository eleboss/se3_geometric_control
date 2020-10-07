clc;clear;close all;
% path = ginput() * 100.0
% disp(path)
path = [0, 0, 0; 
        1, 1, 1; 
];

% path = [9, 9, 9; 
%         10, 10, 10; 
% ];
% path = [11, 11, 11; 
%         10, 10, 10; 
% ];
disp(path);

n_order = 7;
n_seg = size(path, 1) - 1;
n_poly_perseg = n_order + 1;
g = 9.806; % gravity constant

ts = zeros(n_seg, 1);
% calculate time distribution based on distance between 2 points
dist = zeros(n_seg, 1);
dist_sum = 0;

% T = 25;
% t_sum = 0;
% for i = 1:n_seg
%     dist(i) = sqrt((path(i+1, 1) - path(i, 1))^2 + (path(i+1, 2) - path(i, 2))^2);
%     dist_sum = dist_sum + dist(i);
% end
% for i = 1:n_seg-1
%     ts(i) = dist(i) / dist_sum * T;
%     t_sum = t_sum + ts(i);
% end
% ts(n_seg) = T - t_sum;
% or you can simply average the time
for i = 1:n_seg
    ts(i) = 2.0;
end

acc_cond_x = 1;
acc_cond_y = 0;
acc_cond_z = -g + 3;
start_acc_cond_x = 0;
start_acc_cond_y = 0;
start_acc_cond_z = 0.1;




% fprintf('Coef_x \n \n');
poly_coef_x = MinimumSnapCloseformSolver(path(:, 1), acc_cond_x, start_acc_cond_x, ts, n_seg, n_order);
% fprintf('Coef_y \n \n');
poly_coef_y = MinimumSnapCloseformSolver(path(:, 2), acc_cond_y, start_acc_cond_y, ts, n_seg, n_order);
% fprintf('Coef_z \n \n');
poly_coef_z = MinimumSnapCloseformSolver(path(:, 3), acc_cond_z, start_acc_cond_z, ts, n_seg, n_order);

X_n = [];
Xv_n = [];
Xa_n = [];
Y_n = [];
Yv_n = [];
Ya_n = [];
Z_n = [];
Zv_n = [];
Za_n = [];

r3_n = [];
r2_n = [];
r1_n = [];
yaw = 0.0;
fd_n = [];
ad_n = [];

zw = [0.0, 0.0, 1.0];
r2c = [-sin(yaw),cos(yaw),0.0]';

k = 1;
tstep = 0.01;
for i=0:n_seg-1
    %#####################################################
    % STEP 4: get the coefficients of i-th segment of both x-axis, y-axis and z-axis 
    Pxi = [];
    Vxi = [];
    Axi = [];

    Pyi = [];
    Vyi = [];
    Ayi = [];

    Pzi = [];
    Vzi = [];
    Azi = [];
    
    Pxi = flipud(poly_coef_x(1 + i*(n_order+1): (n_order+1) + i*(n_order+1),1) );
    Pxi'
    Vxi = polyder(Pxi);
    Axi = polyder(Vxi);

    Pyi = flipud(poly_coef_y(1 + i*(n_order+1): (n_order+1) + i*(n_order+1),1) );
    Pyi'
    Vyi = polyder(Pyi);
    Ayi = polyder(Vyi);

    Pzi = flipud(poly_coef_z(1 + i*(n_order+1): (n_order+1) + i*(n_order+1),1) );
    Pzi'
    Vzi = polyder(Pzi);
    Azi = polyder(Vzi);

    for t=0:tstep:ts(i+1)
        X_n(k)  = polyval(Pxi,t);
        Xv_n(k)  = polyval(Vxi,t);
        Xa_n(k)  = polyval(Axi,t);

        Y_n(k)  = polyval(Pyi,t);
        Yv_n(k)  = polyval(Vyi,t);
        Ya_n(k)  = polyval(Ayi,t);

        Z_n(k)  = polyval(Pzi,t);
        Zv_n(k)  = polyval(Vzi,t);
        Za_n(k)  = polyval(Azi,t);

        ad_n(k,1) = Xa_n(k);
        ad_n(k,2) = Ya_n(k);
        ad_n(k,3) = Za_n(k);
        fd_n(k,:) = ad_n(k,:) + g * zw;
        r3_n(k,:) = fd_n(k,:)/norm(fd_n(k,:));
        r1_n(k,:) = cross(r2c, r3_n(k,:))/norm(cross(r2c,r3_n(k,:)));
        r2_n(k,:) = cross(r3_n(k,:),r1_n(k,:));

        k = k+1;
    end
end

plot3(X_n, Y_n, Z_n,'Color',[0 1.0 0],'LineWidth',2);
hold on
scatter3(path(1:size(path,1),1),path(1:size(path,1),2),path(1:size(path,1),3));
% Display the drone body vector in each scatter point
for i=1:3:k-1
    x_world = X_n(i);
    y_world = Y_n(i);
    z_world = Z_n(i);
    
    % draw r1
    hold on
    start_array = [x_world, y_world, z_world];
    end_array = [x_world + r1_n(i,1), y_world + r1_n(i,2), z_world + r1_n(i,3)];
    plot3([start_array(1) end_array(1)],[start_array(2) end_array(2)],[start_array(3) end_array(3)],'LineWidth',1);

    % draw r2
    hold on
    start_array = [x_world, y_world, z_world];
    end_array = [x_world + r2_n(i,1), y_world + r2_n(i,2), z_world + r2_n(i,3)];
    plot3([start_array(1) end_array(1)],[start_array(2) end_array(2)],[start_array(3) end_array(3)],'LineWidth',1);

    % draw r3
    hold on
    start_array = [x_world, y_world, z_world];
    end_array = [x_world + r3_n(i,1), y_world + r3_n(i,2), z_world + r3_n(i,3)];
    plot3([start_array(1) end_array(1)],[start_array(2) end_array(2)],[start_array(3) end_array(3)],'LineWidth',1);

end

% final pose
disp("r1");
r1_n(end,:)
disp("r2");
r2_n(end,:)
disp("r3");
r3_n(end,:)

function poly_coef = MinimumSnapCloseformSolver(waypoints, acc_cond, start_acc_cond, ts, n_seg, n_order)
    start_cond = [waypoints(1), 0, start_acc_cond, 0];
    end_cond =   [waypoints(end), 0, acc_cond, 0];
    %#####################################################
    % you have already finished this function in hw1
    Q = getQ(n_seg, n_order, ts);
    %#####################################################
    % STEP 1: compute M
    M = getM(n_seg, n_order, ts);
    % disp(M);
    %#####################################################
    % STEP 2: compute Ct
    Ct = getCt(n_seg, n_order);
    % Ct
    C = Ct';
    R = C * inv(M)' * Q * inv(M) * Ct;
    R_cell = mat2cell(R, [n_seg+7 3*(n_seg-1)], [n_seg+7 3*(n_seg-1)]);
    R_pp = R_cell{2, 2};
    R_fp = R_cell{1, 2};
    % %#####################################################
    % % STEP 3: compute dF
    dF = zeros(n_seg+7);
    dF(1:4) = start_cond(1:4);
    dF(n_seg+7-3:n_seg+7) = end_cond(1:4);
    dF(5:n_seg+7-4) = waypoints(2:(end-1)); 
    dP = -inv(R_pp) * R_fp' * dF;
    poly_coef = inv(M) * Ct * [dF;dP];

end