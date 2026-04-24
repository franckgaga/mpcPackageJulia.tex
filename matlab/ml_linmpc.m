% Executed on:
% MATLAB Version: 24.1.0.3050227 (R2024a) Update 8
% Operating System: Linux 6.17.0

G = [ tf(1.90, [18, 1]) tf(1.90, [18, 1]);
      tf(-0.74,[8, 1])  tf(0.74, [8, 1]) ];
Ts = 2.0;
model = ss(c2d(G, Ts, "zoh"));
A = model.A;
B = model.B;
C = model.C;
yop = [50; 30];
uop = [20; 20];
mpcverbosity off;
Ahat = [ 
    0.894839  0.0       -0.0  -0.0;
    0.0       0.778801   0.0   0.0;
    0.0       0.0        1.0   0.0;
    0.0       0.0        0.0   1.0
];
Bhat = [ 
    -0.334619  -0.334619;
    0.625646  -0.625646;
    0.0        0.0;
    0.0        0.0
];
Chat = [ 
    -0.597112   0.0      1.0  0.0;
    0.0       -0.26163  0.0  1.0
];
mpcmodel = ss(Ahat,Bhat,Chat,[],Ts);
Khat =  [
    -0.0439536   0.0;
    0.0        -0.0198433;
    0.606234    0.0;
    0.0         0.61571;
];

mympc = mpc(mpcmodel,Ts,10,2);
setEstimator(mympc,"custom")
setoutdist(mympc, "model", []);
mympc.OV(1).Min = 45;
mympc.Model.Nominal.U = [20,20];
mympc.Model.Nominal.Y = [50,30];
mympc.Weights.MVrate = [sqrt(0.1) sqrt(0.1)];
myestim = mpcstate(mympc);

figure()
myestim.Plant = [];
myestim.Disturbance = [];
myestim.LastMove = [];
[U_data, Y_data, R_data] = test_mpc(mympc, myestim, ...
    Khat, Ahat, Bhat, Chat, A, B, C, uop, yop);
N = size(U_data, 2);
t_data = (0:N-1)*Ts;
subplot(311)
plot(t_data,Y_data(1,:), t_data, R_data(1,:), "--", t_data, 45*ones(1, N)); grid on;
subplot(312)
plot(t_data,Y_data(2,:), t_data, R_data(2,:), "--"); grid on;
subplot(313)
stairs(t_data,U_data(1,:)); grid on;
hold on;
stairs(t_data, U_data(2,:)); grid on;
hold off;

mympc.Optimizer.Algorithm = "admm";
f = @() test_mpc(mympc, myestim, ...
    Khat, Ahat, Bhat, Chat, A, B, C, uop, yop);
btime_solver_OS = repeated_timeit(f, 500) %#ok<NOPTS> 

mympc.Optimizer.Algorithm = "active-set";
f = @() test_mpc(mympc, myestim, ...
    Khat, Ahat, Bhat, Chat, A, B, C, uop, yop);
btime_solver_AS = repeated_timeit(f, 500) %#ok<NOPTS> 

Ahat = [
    0.894769    0.0          4.73834e-5  0.0          0.0  0.0;
    0.0         0.778067     0.0         0.000460911  0.0  0.0;
    4.73834e-5  0.0          0.894808    0.0          0.0  0.0;
    0.0         0.000460911  0.0         0.778511     0.0  0.0;
    0.0         0.0          0.0         0.0          1.0  0.0;
    0.0         0.0          0.0         0.0          0.0  1.0;
];

Bhat = [ 
    -0.186233  -0.186233     0.39354252250196353;
    0.332939  -0.332939      0.7525766947068777;
    -0.278006  -0.278006    -0.26363045022290316;
    0.529702  -0.529702     -0.4730240748357257;
    0.0        0.0           0.0;
    0.0        0.0           0.0;
 ];
Chat = [
    0.0  0.0  -0.718709   0.0       1.0  0.0;
    0.0  0.0   0.0       -0.309018  0.0  1.0;
];
Dhat = [
    0.0    0.0    0.09999999999999999;
    0.0    0.0     0.08222222222222224;
];
Bhatu = Bhat(:, 1:2);
Bhatd = Bhat(:, 3);
Dhatd = Dhat(:, 3);
yop = [50; 30];
uop = [20; 20];
dop = 20;

Khat = [
    -5.35588e-8   0.0;
    0.0          4.48124e-8;
    -0.013559     0.0;
    0.0         -0.00588797;
    0.613667     0.0;
    0.0          0.61722;
];
mpcmodel_d = ss(Ahat,Bhat,Chat,Dhat,Ts);
mpcmodel_d = setmpcsignals(mpcmodel_d,'MV',[1 2],'MD',3); 
mympc_d = mpc(mpcmodel_d,Ts,10,2);
setEstimator(mympc_d,"custom")
setoutdist(mympc_d, "model", []);
mympc_d.OV(1).Min = 45;
mympc_d.Model.Nominal.U = [20,20,20];
mympc_d.Model.Nominal.Y = [50,30];
mympc_d.Weights.MVrate = [sqrt(0.1) sqrt(0.1)];
myestim_d = mpcstate(mympc_d);


figure()
myestim_d.Plant = [];
myestim_d.Disturbance = [];
myestim_d.LastMove = [];
[U_data, Y_data, R_data] = test_mpc_d(mympc_d, myestim_d, ...
    Khat, Ahat, Bhatu, Bhatd, Chat, Dhatd, A, B, C, uop, yop, dop);

N = size(U_data, 2);
t_data = (0:N-1)*Ts;
subplot(311)
plot(t_data,Y_data(1,:), t_data, R_data(1,:), "--", t_data, 45*ones(1, N)); grid on;
subplot(312)
plot(t_data,Y_data(2,:), t_data, R_data(2,:), "--"); grid on;
subplot(313)
stairs(t_data,U_data(1,:)); grid on;
hold on;
stairs(t_data, U_data(2,:)); grid on;
hold off;

mympc_d.Optimizer.Algorithm = "admm";
f_d = @() test_mpc_d(mympc_d, myestim_d, ...
    Khat, Ahat, Bhatu, Bhatd, Chat, Dhatd, A, B, C, uop, yop, dop);
btime_d_solver_OS = repeated_timeit(f_d, 500) %#ok<NOPTS> 

mympc_d.Optimizer.Algorithm = "active-set";
f_d = @() test_mpc_d(mympc_d, myestim_d, ...
    Khat, Ahat, Bhatu, Bhatd, Chat, Dhatd, A, B, C, uop, yop, dop);
btime_d_solver_AS = repeated_timeit(f_d, 500) %#ok<NOPTS> 

function [U_data, Y_data, R_data] = test_mpc(mympc, myestim, ...
    Khat, Ahat, Bhat, Chat, A, B, C, uop, yop)
    N=75;
    r = [50,30];
    ul = 0;
    Y_data = zeros(2,N);
    U_data = zeros(2,N);
    R_data = zeros(2,N);
    x = zeros(size(A, 1), 1);
    y = C*x + yop;
    u = uop;
    xhat = [(eye(4) - Ahat); Chat]\[Bhat*(u-uop); (y-yop)];
    for i=1:N
        if i == 26
            r = [48,35];
        end
        if i == 51
            ul = -10;
        end
        y = C*x + yop;
        xhat = xhat + Khat*((y-yop) - Chat*xhat);
        myestim.Plant = xhat;
        u = mpcmove(mympc, myestim, [], r);
        U_data(:,i) = u;
        Y_data(:,i) = y;
        R_data(:,i) = r;
        x = A*x + B*(u-uop+[0; ul]);
        xhat = Ahat*xhat + Bhat*(u-uop);
    end
end

function [U_data, Y_data, R_data] = test_mpc_d(mympc, myestim, ...
    Khat, Ahat, Bhatu, Bhatd, Chat, Dhatd, A, B, C, uop, yop, dop)
    N=75;
    r = [50,30];
    ul = 0;
    Y_data = zeros(2,N);
    U_data = zeros(2,N);
    R_data = zeros(2,N);
    x = zeros(size(A, 1), 1);
    y = C*x + yop;
    u = uop;
    d = dop;
    xhat = [(eye(6) - Ahat); Chat] \ ...
           [Bhatu*(u-uop)+Bhatd*(d-dop); (y-yop)-Dhatd*(d-dop)];
    for i=1:N
        if i == 26
            r = [48,35];
        end
        if i == 51
            ul = -10;
        end
        y = C*x + yop;
        d = ul + dop;
        xhat = xhat + Khat*((y-yop) - Chat*xhat - Dhatd*(d-dop));
        myestim.Plant = xhat;
        u = mpcmove(mympc, myestim, [], r, d);
        U_data(:,i) = u;
        Y_data(:,i) = y;
        R_data(:,i) = r;
        x = A*x + B*(u-uop+[0; ul]);
        xhat = Ahat*xhat + Bhatu*(u-uop) + Bhatd*(d-dop);
    end
end