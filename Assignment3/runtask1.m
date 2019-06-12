%% Information 
% This file is only an example of how you can start the simulation. The
% sampling time decides how often you store states. The execution  time
% will increase if you reduce the sampling time.

% Please note that the file "pathplotter.m" (only used in the second part
% of the assignment) shows the ship path during the path following and
% target tracking part of the assignment. It can be clever to adjust the sampling
% time when you use that file because it draws a sketch of the ship in the
% North-East plane at each time instant. Having a small sampling time will
% lead to multiple ship drawings on top of each other. 

% You should base all of your simulink models on the MSFartoystyring model
% and extend that as you solve the assignment. For your own sake, it is
% wise to create a new model and run file for each task.

% The msfartoystyring.m file includes the ship model. You are not allowed
% to change anything within that file. You need to include that file in
% every folder where you have a simulink model based on
% "MSFartoystyring.slx". 

% WP.mat is a set of six waypoints that you need to use in the second part of
% the assignment. The north position is given in the first row and the east
% position in the second row. 
deg2rad = pi/180;
rad2deg = 180/pi;
%%


psi_d = 25*deg2rad;
n_c = 7.3;
n_c_max = 85*2*pi/60;
delta_c_max = 25*deg2rad;
u_max = 8.6075;

tstart=0;           % Sim start time
tstop=8500;        % Sim stop time
tsamp=10;           % Sampling time for how often states are stored. (NOT ODE solver time step)
                
p0=[1000, 700];      % Initial position (NED)
v0=[.1 0]';       % Initial velocity (body)
psi0=0*deg2rad;    % Inital yaw angle
r0=0;               % Inital yaw rate
c=1;                % Current on (1)/off (0)


%% Controller
K = -0.025;
T = 40;

zeta = 0.5;
wb = 0.1;
wn = 1/(sqrt(1-2*zeta^2 + sqrt(4*zeta^4 - 4*zeta^2+2)))*wb;
Km = 1;
m = T/K;
d = 1/K;
k = 0;

kp = -(m + Km)*wn^2-k;
kd = -2*zeta*wn*(m + Km) + d;
ki = (wn/10)*kp;

lambda_u = 0.01;
kp_u = 2*lambda_u;
ki_u = lambda_u^2;
ki_u = 0.000;

d1 = -0.0021;
d2 = 1.0377;
m = 5500;

% Reference Model params
zeta = 1;
omega_psi = 0.1;
omega_u = 0.008;

%% Curvefit u and n_c

% n_c_test = 0.42*n_c_max;
n = [0 ,0.1*n_c_max, 0.2*n_c_max, 0.3*n_c_max, 0.4*n_c_max, 0.5*n_c_max, 0.6*n_c_max, 0.7*n_c_max, 0.8*n_c_max, 0.9*n_c_max, 1*n_c_max];
u = [0, 0.2985, 0.5957, 0.8965, 1.2314, 4.1050, 5.0234, 5.9288, 6.8264, 7.7188, 8.6075];
% %u_max = u(end);

% %u_d = 2;
% 
% resulting_n_c = f(1)*u_d^10 + f(2)*u_d^9 + f(3)*u_d^8 + f(4)*u_d^7 + f(5)*u_d^6 + f(6)*u_d^5 + f(7)*u_d^4 + f(8)*u_d^3 + f(9)*u_d^2 + f(10)*u_d + f(11)
% figure(10); clf;hold on;
% scatter(u,n);
% scatter(u_d, resulting_n_c);
%plot(f, n, u)
syms d1 d2;
u(3) = 2.9608;
n(3) = 3;
u(7) = 6.8727;
n(7) = 7;
eq1 = -d1*u(3) - d2*u(3)^2 == n(3)^2;
eq2 = -d1*u(7) - d2*u(7)^2 == n(7)^2;
eq3 = -d1*u(5) - d2*u(5)^2 == n(5)^2;

s = solve([eq1, eq2], [d1, d2]);
d1 = double(s.d1);
d2 = double(s.d2);
d1 = -d1
d2 = -d2
u_dot = 0;
u = 0;
m_test = 500;
tim = 0:0.1:n_c_max;
table = zeros(length(tim), 2);
idx = 1;
for i = tim
    u_dot = 1/m_test*(i^2 - d1*u - d2*u^2);
    u = u + u_dot*0.1;
    table(idx, 1) = u;
    table(idx, 2) = i;
    idx = idx + 1;
end

figure();clf;
plot(n_c_test, u_test, 'r')




%% Sim
sim MSFartoystyring2017b % The measurements from the simulink model are automatically written to the workspace.

%% Plot


% chi = zeros(length(p(:,1)),1);
% for i = 1:length(p(:,1))
%     chi(i) = atan2(p(i,1),p(i,2));
% end
% 
figure(1); clf;hold on; grid on;
plot(t, psi*rad2deg, 'b');
% plot(t, psi_d*rad2deg, '-r');
% plot(t, (psi - psi_d)*rad2deg, 'k');
legend('\psi', '\psi_d', '\psi - \psi_d');
title('heading')

figure(2); clf;hold on; grid on;
plot(t, r, 'b');
% plot(t, r_d, '-r');
% plot(t, r - r_d, 'k');
legend('r', 'r_d', 'r - r_d');
title('Yaw Rate')
% 
% 
% 
figure(3); clf;hold on; grid on;
plot(t, u, 'b');
plot(t, u_d, 'g');
plot(t, u - u_d, 'k');
legend('u','u_d','u - u_d');
title('Surge')

% 
% figure(4); clf;hold on; grid on;
% plot(p(:,2), p(:,1), 'r');
% legend('North', 'East');
% 
% 
figure(5); clf;hold on; grid on;
plot(t, n_c)
legend('n_c');
title('propeller')
% 
figure(6); clf; hold on; grid on;
plot(t, delta_c);
legend('\delta_c')
title('rudder')
% 
% figure(7); clf; hold on; grid on;
% plot(t, chi*rad2deg, 'b');
% plot(t, psi*rad2deg, 'r');
% plot(t, (chi - psi)*rad2deg, 'g');
% legend('\chi', '\psi', '\chi - \psi','\beta');
% ylabel('deg')
% xlabel('t')
% title('Course, Heading and Sideslip')