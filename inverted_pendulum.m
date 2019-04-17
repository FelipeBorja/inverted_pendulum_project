% Constants
L = 0.5; % in m
g = 9.8; % in m/s^2

% State-space matrices
A = [0 1 0 0;(g/L) 0 0 0; 0 0 0 1; 0 0 0 0];
B = [0; (1/L); 0; 1];
C = [1 0 0 0; 0 0 1 0];
D = [0; 0];

% Establish stability, controllability
A_eig = eig(A); % stable if eigenvalues < 0
Mc = [B (A*B) (A^2*B) (A^3*B)]; % controllability matrix
rank(Mc); % controllable if full rank
Mo = [C; (C*A); (C*A^2); (C*A^3)]; % observability matrix
rank(Mo); % observable if full rank

% only need integral control on angular displacement
C_top = C(1,:);
D_top = D(1,:);

% Part 3: Design state fb control sys with integral action
A_aug = [0 -C_top; zeros(4,1) A];
B_aug = [-D_top; B];
Br_aug = [1; zeros(4,1)];
Bw_aug = B_aug;

Mc_aug = [B_aug (A_aug*B_aug) (A_aug^2*B_aug) (A_aug^3*B_aug)]; % controllability matrix
rank(Mc_aug) % controllable if full rank

% Placing poles
Kbar = place(A_aug, B_aug, [-5 -16 -27 -38 -49])
A_f = [-D]


