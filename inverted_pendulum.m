% inverted_pendulum.m
% Felipe Borja and Casey Gardner
% E102 Midterm Project

%% Part 0: Defining Constants
% Constants
L = 0.5; % in m
g = 9.8; % in m/s^2
alpha = 0.5; % in rad/s^2
ICo = 0; % observer initial condition

%% Part 1: Defining the State Space Matrices
% State-space matrices
A = [0 1 0 0;(g/L) 0 0 0; 0 0 0 1; 0 0 0 0];
B = [0; -(1/L); 0; 1];
C = [1 0 0 0; 0 0 1 0];
D = [0; 0];

%% Part 2: Establish stability, controllability, observability
disp(' ')
A_eig = eig(A); % stable if eigenvalues < 0
if (max(A_eig) <= 0); disp('Original system is stable')
else disp('Original system is UNstable')
end

Mc = [B (A*B) (A^2*B) (A^3*B)]; % controllability matrix
%controllabe if full rank
if(rank(Mc) == size(Mc,2)); disp('System is controllable');
else disp('System is NOT controllable')
end

Mo = [C; (C*A); (C*A^2); (C*A^3)]; % observability matrix
% observable if full rank
if (rank(Mo)==size(Mo,2)); disp('System is observable'); 
else disp('System is NOT observable')
end

%% Part 3.1: Design integral action sys with feedback
% only need integral control on translational displacement
%       disturbance is angular, but we are controlling translation
C_bot = C(2,:);
D_bot = D(2,:);
%calculate augmented matrices
A_aug = [0 -C_bot; zeros(4,1) A];
B_aug = [-D_bot; B];
Br_aug = [1; zeros(4,1)];
Bw_aug = B_aug;
C_aug = [C zeros(2,1)];

%Mc_aug = [B_aug (A_aug*B_aug) (A_aug^2*B_aug) (A_aug^3*B_aug) (A_aug^4*B_aug)] % controllability matrix
%rank(Mc_aug) % controllable if full rank

% placing poles based on 2nd-order (so) dominant behavior
wn = 1.3; %nat frequency
zeta = 1.1; %damping ratio
[ sop1, sop2, so_respInfo ] = secondOrderStep( wn, zeta, 1 );
% pole placements
pfactor = 2;
pki = [-15 sop1 sop2 pfactor*sop1 pfactor*sop2];
p1 = pki(1,2);
p2 = pki(1,3);
p3 = pki(1,4);
p4 = pki(1,5);

% finding controller kbar
kpoles = place(A_aug, B_aug, pki);
%extract integral controller gain
ki = -kpoles(1);
kbar = kpoles(2:end);
% feedback matrix, to check correct pole placement
A_f = [-D_bot*ki    -C_bot + D_bot*kbar; ...
        B*ki                A-B*kbar ];
% check stability of feedback matrix
newpol = eig(A_f);
if (max(newpol) <= 0); disp('Feedback system is stable')
else disp('Feedback system is UNstable')
end
disp(' ')

% plot step response for four chosen poles
s = tf('s');
tcl = 1/( (s-p1)*(s-p2)*(s-p3)*(s-p4) );
[y, t] = step(tcl);
%figure
plot(t,y)
xlabel('Time (s)')
ylabel('Unit Step Response')
title(sprintf('Step Response with Chosen Poles: [%.2f, %.2f, %.2f, %.2f]',... 
    p1, p2, p3, p4))
grid on
stats = stepinfo(tcl, 'SettlingTimeThreshold', 0.001);

%% Part 3.2: Design observer
% observer poles are an order of magnitude higher
pe = 10*pki(1,2:end);
lbar = place(A', C', pe)';


sim('pendulum_control_linear')
figure(2)
clf
plot(tout, yout.signals(1).values')
hold on
plot(tout, yout.signals(2).values')
plot(tout, yout.signals(3).values')
legend('th', 's', 'a')
xlim([0 6])
ylim([-1 1])

sim('pendulum_control')