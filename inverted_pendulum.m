% Constants
L = 0.5; % in m
g = 9.8; % in m/s^2
 
% State-space matrices
A = [0 1 0 0;(g/L) 0 0 0; 0 0 0 1; 0 0 0 0];
B = [0; -(1/L); 0; 1];
C = [1 0 0 0; 0 0 1 0];
D = [0; 0];

% Establish stability, controllability
A_eig = eig(A); % stable if eigenvalues < 0
Mc = [B (A*B) (A^2*B) (A^3*B)]; % controllability matrix
rank(Mc); % controllable if full rank
Mo = [C; (C*A); (C*A^2); (C*A^3)]; % observability matrix
rank(Mo); % observable if full rank
 
pk = [-3 -6 -9 -12];
clkbar = place(A, B, pk)

pl = [-15 -20 -25 -30];
cllbar = transpose(place(A', C', pl))

% Part 3: Design state fb control sys with integral action
% only need integral control on translational displacement?
C_bot = C(2,:);
D_bot = D(2,:);
%calculate augmented matrices
A_aug = [0 -C_bot; zeros(4,1) A];
B_aug = [-D_bot; B];
Br_aug = [1; zeros(4,1)];
Bw_aug = B_aug;

%Mc_aug = [B_aug (A_aug*B_aug) (A_aug^2*B_aug) (A_aug^3*B_aug) (A_aug^4*B_aug)] % controllability matrix
%rank(Mc_aug) % controllable if full rank

% Placing poles
%syms ki k1 k2 k3 k4 s
%A_f = [-D_bot*ki -C_bot + D_bot*[k1 k2 k3 k4]; B*ki A-B*[k1 k2 k3 k4]];
%char = det(s*eye(5) - A_f);
kbar = place(A_aug, B_aug, [-50 -67 -89 -99 -130]);
A_f = [-D_bot*kbar(1,1) -C_bot + D_bot*kbar(1,2:end); ...
        B*kbar(1,1) A-B*kbar(1,2:end)];
eig_aug = eig(A_f)