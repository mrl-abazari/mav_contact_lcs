function [F_bar,q] = lcs_gen_kbf(x,u,mu,b,k)

% this function receives the quadcopter system states as a vector x = [phi;
% theta; psi; Z; X; Y; d_phi; d_theta; d_psi; dZ; dX; dY] and generates the
% LCS (Linear Complementarity System) constraints matrix F and the vector q 
% to feed the LCP Solver. u is the system input vector [U1; U2; U3; U4]
% the general form of the LCS constrints formulation is:   E.x + H.u + F.lambda + c 
% where the LCP(F,q) is to be solved to obtain "lambda", q = E.x + H.u + c

%%%   The collision case with Stiffness, Damping, and Friction   %%%

% initializing the constants 

l = 0.12;       % [m]   drone [rolling/pitching] arm length
rho = 0.09;     % [m]   propeller guard radius
p = 0.83;     % [m]   the location of the colliding wall (X = p) in the Earth-fixed inertial frame % 0.8448
M = 100.000;    % [] slack variable to capture the effect of the damper


% assigning the system state variables

phi     = x(1,1);
theta   = x(2,1);
psi     = x(3,1);
Z       = x(4,1);
X       = x(5,1);
Y       = x(6,1);
d_phi   = x(7,1);
d_theta = x(8,1);
d_psi   = x(9,1);
d_Z     = x(10,1);
d_X     = x(11,1);
d_Y     = x(12,1);

% calculating the intermediate variables

C11   = cos(psi)*cos(theta);
d_C11 = -d_psi*sin(psi)*cos(theta) - d_theta*cos(psi)*sin(theta);
C12   = cos(psi)*sin(theta)*sin(phi) - sin(psi)*cos(phi);
d_C12 = -d_psi*sin(psi)*sin(theta)*sin(phi) - d_psi*cos(psi)*cos(phi)...
    + d_theta*cos(psi)*cos(theta)*sin(phi)...
    + d_phi*cos(psi)*sin(theta)*cos(phi) + d_phi*sin(psi)*sin(phi);
C13   = cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi);
d_C13 = -d_psi*sin(psi)*sin(theta)*cos(phi) + d_psi*cos(psi)*sin(phi)...
    + d_theta*cos(psi)*cos(theta)*cos(phi)...
    - d_phi*cos(psi)*sin(theta)*sin(phi) + d_phi*sin(psi)*cos(phi);
C21   = sin(psi)*cos(theta);
d_C21 = d_psi*cos(psi)*cos(theta) - d_theta*sin(psi)*sin(theta);
C22   = sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi);
d_C22 = d_psi*cos(psi)*sin(theta)*sin(phi) - d_psi*sin(psi)*cos(phi)...
    + d_theta*sin(psi)*cos(theta)*sin(phi)...
    + d_phi*sin(psi)*sin(theta)*cos(phi) - d_phi*cos(psi)*sin(phi);
C23   = sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi);
d_C23 = d_psi*cos(psi)*sin(theta)*cos(phi) + d_psi*sin(psi)*sin(phi)...
    + d_theta*sin(psi)*cos(theta)*cos(phi)...
    - d_phi*sin(psi)*sin(theta)*sin(phi) - d_phi*cos(psi)*cos(phi);
C31   = -sin(theta);
d_C31 = -d_theta*cos(theta);
C32   = cos(theta)*sin(phi);
d_C32 = -d_theta*sin(theta)*sin(phi) + d_phi*cos(theta)*cos(phi);
C33   = cos(theta)*cos(phi);
d_C33 = -d_theta*sin(theta)*cos(phi) - d_phi*cos(theta)*sin(phi);

% defining the rotor center positions in the body-fixed frame

xm = [+l, +l, -l, -l];
ym = [-l, +l, +l, -l];
zm = [0,  0,  0,  0 ];

% defining the Complementarity Condition matrices for a collising wall with
% contact stiffenss k, damping b, and dynamic coefficient of friction mu



for i = 1:4

    F(:,:,i) = [1 1 0 0 0 0 0;...
        0 1 0 0 0 0 0;...
        0 0 0 0 0 0 1;...
        0 0 0 0 0 0 1;...
        0 0 0 0 0 0 1;...
        0 0 0 0 0 0 1;...
        mu 0 -1 -1 -1 -1 0];

    E(:,:,i) = [0 0 0 0 -k 0 0 0 0 0 -b 0;...
        0 0 0 0 M 0 0 0 0 0 0 0;...
        0 0 0 0 0 0 0 0 0 0 0 1;...
        0 0 0 0 0 0 0 0 0 1 0 0;...
        0 0 0 0 0 0 0 0 0 0 0 -1;...
        0 0 0 0 0 0 0 0 0 -1 0 0;...
        0 0 0 0 0 0 0 0 0 0 0 0];

    H(:,:,i) = zeros(7,4);

    c(:,i) = [-k*(C11*xm(i)+C12*ym(i)+C13*zm(i))-k*rho-b*(d_C11*xm(i)+d_C12*ym(i)+d_C13*zm(i))+k*p;...
        M*rho-M*p;...
        d_C21*xm(i)+d_C22*ym(i)+d_C23*zm(i);...
        d_C31*xm(i)+d_C32*ym(i)+d_C33*zm(i);...
        -d_C21*xm(i)-d_C22*ym(i)-d_C23*zm(i);...
        -d_C31*xm(i)-d_C32*ym(i)-d_C33*zm(i);
        0];

end

F_bar = blkdiag(F(:,:,1),F(:,:,2),F(:,:,3),F(:,:,4));

E_bar = [E(:,:,1);E(:,:,2);E(:,:,3);E(:,:,4)];

H_bar = [H(:,:,1);H(:,:,2);H(:,:,3);H(:,:,4)];

c_bar = [c(:,1);c(:,2);c(:,3);c(:,4)];

q = E_bar*x + H_bar*u + c_bar;

end