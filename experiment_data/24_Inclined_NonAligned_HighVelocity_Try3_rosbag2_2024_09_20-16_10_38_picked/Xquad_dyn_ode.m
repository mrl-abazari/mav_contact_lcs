function [dx] = Xquad_dyn_ode(t,x,U,lambda)


% this function receives the time t, system state vector x = [phi;
% theta; psi; Z; X; Y; d_phi; d_theta; d_psi; dZ; dX; dY], system input
% vector U, and the contact forces vector lambda as inputs and outputs
% the system dynamics as a usable function for ode-based solvers.


% initiating the system constants

l = 0.12;       % [m]   drone [rolling/pitching] arm length
m = 1.234;      % [kg]   drone total weight 
rho = 0.09;     % [m]   propeller guard radius
Ixx = 0.007101; % [kg.m^2]   moment of inertia about the body x axis
Iyy = 0.007106; % [kg.m^2]   moment of inertia about the body y axis
Izz = 0.0137;   % [kg.m^2]   moment of inertia about the body z axis

g = 0;       % [m.s^(-2)]   gravity   % 9.81


% state variables assignment

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

% rotation matrices calculation

R_phi   = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];           % rotation matrix along body x-axis
R_theta = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];   % rotation matrix along bosy y-axis
R_psi   = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];           % rotation matrix along bosy z-axis
R_ptp   = R_psi * R_theta * R_phi;                                      % resultant rotation matrix

% intermediate constants and variables

b1 = 1 / Ixx;
b2 = 1 / Iyy;
b3 = 1 / Izz;

u_z = cos(phi)*cos(theta);
u_x = cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
u_y = cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);

% input assignment

U1 = U(1,1);
U2 = U(2,1);
U3 = U(3,1);
U4 = U(4,1);

% rotor center position vectors represented in the body-fixed frame

r_bb_m(:,1) = [+l; -l; 0];   % rotor center #1 position vector represented in body-fixed frame w.r.t. body-fixed frame
r_bb_m(:,2) = [+l; +l; 0];   % rotor center #2 position vector represented in body-fixed frame w.r.t. body-fixed frame
r_bb_m(:,3) = [-l; +l; 0];   % rotor center #3 position vector represented in body-fixed frame w.r.t. body-fixed frame
r_bb_m(:,4) = [-l; -l; 0];   % rotor center #4 position vector represented in body-fixed frame w.r.t. body-fixed frame

% rotor center position vectors represented in the Earth-fixed inertial frame

r_bE_m(:,1) = R_ptp * r_bb_m(:,1);   % rotor center #1 position vector represented in body-fixed frame w.r.t. Earth-fixed frame
r_bE_m(:,2) = R_ptp * r_bb_m(:,2);   % rotor center #2 position vector represented in body-fixed frame w.r.t. Earth-fixed frame
r_bE_m(:,3) = R_ptp * r_bb_m(:,3);   % rotor center #3 position vector represented in body-fixed frame w.r.t. Earth-fixed frame
r_bE_m(:,4) = R_ptp * r_bb_m(:,4);   % rotor center #4 position vector represented in body-fixed frame w.r.t. Earth-fixed frame

% contact point position vectors represented in the body-fixed frame

r_bb_c(:,1) = [+l+rho; -l; 0];   % contact point #1 position vector represented in body-fixed frame w.r.t. body-fixed frame
r_bb_c(:,2) = [+l+rho; +l; 0];   % contact point #2 position vector represented in body-fixed frame w.r.t. body-fixed frame
r_bb_c(:,3) = [-l+rho; +l; 0];   % contact point #3 position vector represented in body-fixed frame w.r.t. body-fixed frame
r_bb_c(:,4) = [-l+rho; -l; 0];   % contact point #4 position vector represented in body-fixed frame w.r.t. body-fixed frame

% contact point position vectors represented in the Earth-fixed inertial frame

r_bE_c(:,1) = R_ptp * r_bb_c(:,1);   % contact point #1 position vector represented in body-fixed frame w.r.t. Earth-fixed frame
r_bE_c(:,2) = R_ptp * r_bb_c(:,2);   % contact point #2 position vector represented in body-fixed frame w.r.t. Earth-fixed frame
r_bE_c(:,3) = R_ptp * r_bb_c(:,3);   % contact point #3 position vector represented in body-fixed frame w.r.t. Earth-fixed frame
r_bE_c(:,4) = R_ptp * r_bb_c(:,4);   % contact point #4 position vector represented in body-fixed frame w.r.t. Earth-fixed frame


% normal contact forces and moments calculation

lambda_n(:,1) = [-lambda(1,1);  0; 0];   % normal contact force vector for the propeller guard #1
lambda_n(:,2) = [-lambda(8,1);  0; 0];   % normal contact force vector for the propeller guard #2
lambda_n(:,3) = [-lambda(15,1); 0; 0];   % normal contact force vector for the propeller guard #3
lambda_n(:,4) = [-lambda(22,1); 0; 0];   % normal contact force vector for the propeller guard #4

Lambda_n = lambda_n(:,1) + lambda_n(:,2) + lambda_n(:,3) + lambda_n(:,4);   % resultant contact force vector

taw_n(:,1) = cross(r_bE_m(:,1), lambda_n(:,1));   % the moment about the quadcopter CoG resulted from the normal contact force #1
taw_n(:,2) = cross(r_bE_m(:,2), lambda_n(:,2));   % the moment about the quadcopter CoG resulted from the normal contact force #2
taw_n(:,3) = cross(r_bE_m(:,3), lambda_n(:,3));   % the moment about the quadcopter CoG resulted from the normal contact force #3
taw_n(:,4) = cross(r_bE_m(:,4), lambda_n(:,4));   % the moment about the quadcopter CoG resulted from the normal contact force #4

Taw_n = taw_n(:,1) + taw_n(:,2) + taw_n(:,3) + taw_n(:,4);   % the resulant moment about the quadcopter CoG resulted from the all normal conatact forces

% tangential conatct forces and moments calculation

beta_1 = lambda(3:6,1);                                                  % friction vector weights extraction for constraint #1
lambda_t(:,1) = [0; beta_1(1,1)-beta_1(3,1); beta_1(2,1)-beta_1(4,1)];   % tangential friction force vector on the propeller guard #1
beta_2 = lambda(10:13,1);                                                % friction vector weights extraction for constraint #2
lambda_t(:,2) = [0; beta_2(1,1)-beta_2(3,1); beta_2(2,1)-beta_2(4,1)];   % tangential friction force vector on the propeller guard #2
beta_3 = lambda(17:20,1);                                                % friction vector weights extraction for constraint #3
lambda_t(:,3) = [0; beta_3(1,1)-beta_3(3,1); beta_3(2,1)-beta_3(4,1)];   % tangential friction force vector on the propeller guard #3
beta_4 = lambda(24:27,1);                                                % friction vector weights extraction for constraint #4
lambda_t(:,4) = [0; beta_4(1,1)-beta_4(3,1); beta_4(2,1)-beta_4(4,1)];   % tangential friction force vector on the propeller guard #4

Lambda_t = lambda_t(:,1) + lambda_t(:,2) + lambda_t(:,3) + lambda_t(:,4);   % resultant friction force acting on the CoG of the drone

taw_t(:,1) = cross(r_bE_c(:,1), lambda_t(:,1));   % the moment about the quadcopter CoG resulted from the friction force on propeller guard #1
taw_t(:,2) = cross(r_bE_c(:,2), lambda_t(:,2));   % the moment about the quadcopter CoG resulted from the friction force on propeller guard #2
taw_t(:,3) = cross(r_bE_c(:,3), lambda_t(:,3));   % the moment about the quadcopter CoG resulted from the friction force on propeller guard #3
taw_t(:,4) = cross(r_bE_c(:,4), lambda_t(:,4));   % the moment about the quadcopter CoG resulted from the friction force on propeller guard #4

Taw_t = taw_t(:,1) + taw_t(:,2) + taw_t(:,3) + taw_t(:,4);   % the resulant moment about the quadcopter CoG resulted from the all friction conatact forces

% system dynamics (Equations of Motion - EoM)

dd_phi   = b1*U2            + b1*(Taw_n(1,1)+Taw_t(1,1));
dd_theta = b2*U3            + b2*(Taw_n(2,1)+Taw_t(2,1));
dd_psi   = b3*U4            + b3*(Taw_n(3,1)+Taw_t(3,1));
dd_Z     = g - u_z*(1/m)*U1 + (1/m)*(Lambda_n(3,1)+Lambda_t(3,1));
dd_X     = u_x*(1/m)*U1     + (1/m)*(Lambda_n(1,1)+Lambda_t(1,1));
dd_Y     = u_y*(1/m)*U1     + (1/m)*(Lambda_n(2,1)+Lambda_t(2,1));


% output assignment

dx = [d_phi; d_theta; d_psi; d_Z; d_X; d_Y; dd_phi; dd_theta; dd_psi; dd_Z; dd_X; dd_Y];

end