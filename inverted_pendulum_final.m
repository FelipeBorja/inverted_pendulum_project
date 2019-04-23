% inverted_pendulum_final.m
% Felipe Borja and Casey Gardner
% E102 Midterm Project, 23 April 2019

%clear

%% Part 0: Defining Constants
% Define Constants
L = 0.5;        % in m
g = 9.8;        % in m/s^2
alpha = 0.5;    % in rad/s^2
ICo = 0;        % observer initial condition

wn = 0.8;       % nat frequency of secondOrderSys
zeta = 1.01;    % damping ratio of secondOrderSys
pfactor = 5;    % factor for other two system poles
poleki = -8.5;  % integral controller pole

%% Part 1: Defining the State Space Matrices
% Define State-space matrices
A = [0 1 0 0;(g/L) 0 0 0; 0 0 0 1; 0 0 0 0];
B = [0; -(1/L); 0; 1];
C = [1 0 0 0; 0 0 1 0];
D = [0; 0];

%% Part 2: Establish stability, controllability, observability
disp(' ')
A_eig = eig(A);
% stable if eigenvalues < 0
if (max(A_eig) <= 0); disp('Original system is stable')
else; disp('Original system is UNstable')
end

Mc = [B (A*B) (A^2*B) (A^3*B)]; % controllability matrix
%controllable if full rank
if(rank(Mc) == size(Mc,2)); disp('System is controllable');
else; disp('System is NOT controllable')
end

Mo = [C; (C*A); (C*A^2); (C*A^3)]; % observability matrix
% observable if full rank
if (rank(Mo)==size(Mo,2)); disp('System is observable'); 
else; disp('System is NOT observable')
end

%% Part 3.1: Design integral action sys with feedback
% We only need integral control on translational displacement
%       disturbance is angular, but we are controlling translation
%       so we take the bottom row of C and D
C_bot = C(2,:);
D_bot = D(2,:);
% Calculate augmented matrices for integral control
A_aug = [0 -C_bot; zeros(4,1) A];
B_aug = [-D_bot; B];
Br_aug = [1; zeros(4,1)];
Bw_aug = B_aug;
C_aug = [zeros(2,1) C];

% We assume that the behavior of the system is dominated by the
%       behavior of two secondOrderSys poles, sop1 and sop2
[ sop1, sop2, so_respInfo ] = secondOrderStep( wn, zeta, 0 );
% Define poles to place, pki = poles that include ki
pki = [poleki sop1 sop2 pfactor*sop1 pfactor*sop2];
% pki = [-8.5000   -0.6946   -0.9214   -3.4729   -4.6071];
% Extract 4 "system" poles (not poleki) for later evaluation
p1 = pki(1,2);
p2 = pki(1,3);
p3 = pki(1,4);
p4 = pki(1,5);

% Use pole placement to find k_i and k_bar
kpoles = acker(A_aug, B_aug, pki);
% Extract integral controller gain and controller vector
ki = kpoles(1);
kbar = kpoles(2:end);

%{
% Define feedback matrix, to check correct pole placement
A_f = [-D_bot*ki    -C_bot + D_bot*kbar; ...
        B*ki                A-B*kbar ];
% Check the stability of feedback matrix
newpol = eig(A_f);
if (max(real(newpol)) <= 0); disp('Feedback system is stable')
else; disp('Feedback system is UNstable')
end
%}
disp(' ') % nice printing space

%{
% Plot the step response for four chosen "system" poles
%       note that only the "settling time" is accurate
%       to predict full behavior, we'd need the numerator of the TF
s = tf('s');
tcl = 1/( (s-p1)*(s-p2)*(s-p3)*(s-p4) );
[y, t] = step(tcl);
figure(1)
plot(t,y)
xlabel('Time (s)')
ylabel('Unit Step Response')
title(sprintf('Step Response with Chosen Poles: [%.2f, %.2f, %.2f, %.2f]',... 
    p1, p2, p3, p4))
grid on
stats = stepinfo(tcl, 'SettlingTimeThreshold', 0.001);
%}

%% Part 3.2: Design observer
% Observer poles are much faster than controller poles, 
%       not including pole corresponding to ki
% pe = poles of l_bar
pe = 50*pki(1,2:end); 
lbar = place(A', C', pe)';

tspan = 15; % how long to simulate in Simulink

%% Non-linear Simulink Simulation

disp('Simulating nonlinear system...')
sim('pendulum_control_final', tspan)
figure(3)
clf
subplot(3,1,1)
plot(tout, yout.signals(1).values', 'LineWidth', 2)
title('Angular Position (rad)')
xlabel('Time (s)')
grid on

subplot(3,1,2)
plot(tout, yout.signals(2).values', 'LineWidth', 2)
title('Cart Position (m)')
xlabel('Time (s)')
grid on

subplot(3,1,3)
plot(tout, yout.signals(3).values', 'LineWidth', 2)
title('Cart Acceleration (m/s^2)')
xlabel('Time (s)')
grid on
sgtitle({'Non-Linearized System'})
set(gcf, 'color', 'w')

disp(' ') % nice printing space
