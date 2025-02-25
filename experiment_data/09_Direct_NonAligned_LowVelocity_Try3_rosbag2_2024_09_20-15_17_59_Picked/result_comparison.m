%%%   this is an Amirali Abazari developed code   %%%   27 September 2024

%%%   _-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_   %%%

%%%   this code is written to plot the quadcopter collision simulation result
%%%   and compare it with the experiment results around the collision point   %%%

%%%   _-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_   %%%


clc; clear all; close all;

%%% reading the experiment rosbag data

bagData_pose   = readtable("09_pose.csv");
bagData_wrench = readtable("09_wrench.csv");

%%% reading and assigning the required variable values from inside the rosbags

t_p_full_array_not_calibrated = bagData_pose.x__time;   % reading the position data time array

t_p_full_array_lin = linspace(t_p_full_array_not_calibrated(1), t_p_full_array_not_calibrated(end),...
    length(t_p_full_array_not_calibrated));  % making linearly spaced time stamps to avoid precision errors due to unevenly distributed data points

for i = 1:length(t_p_full_array_lin)

    t_p_full_array_calibrated(i) = t_p_full_array_lin(i) - t_p_full_array_lin(1);   % zero-ing the first time stamp

end

X_full_array = interpNaN(t_p_full_array_calibrated, transpose(bagData_pose.x_vrpn_mocap_RigidBody1_pose_pose_position_x));   % reading the drone CoG world-frame X + Interpolating the NaN values
Y_full_array = interpNaN(t_p_full_array_calibrated, transpose(bagData_pose.x_vrpn_mocap_RigidBody1_pose_pose_position_y));   % reading the drone CoG world-frame Y + Interpolating the NaN values
Z_full_array = interpNaN(t_p_full_array_calibrated, transpose(bagData_pose.x_vrpn_mocap_RigidBody1_pose_pose_position_z));   % reading the drone CoG world-frame Z + Interpolating the NaN values

phi_full_array   = interpNaN(t_p_full_array_calibrated, transpose(bagData_pose.x_vrpn_mocap_RigidBody1_pose_pose_orientation_roll));    % reading the drone roll  angle + Interpolating the NaN values
theta_full_array = interpNaN(t_p_full_array_calibrated, transpose(bagData_pose.x_vrpn_mocap_RigidBody1_pose_pose_orientation_pitch));   % reading the drone pitch angle + Interpolating the NaN values
psi_full_array   = interpNaN(t_p_full_array_calibrated, transpose(bagData_pose.x_vrpn_mocap_RigidBody1_pose_pose_orientation_yaw));     % reading the drone yaw   angle + Interpolating the NaN values

d_X_full_array(1) = 0; d_Y_full_array(1) = 0; d_Z_full_array(1) = 0;
d_phi_full_array(1) = 0; d_theta_full_array(1) = 0; d_psi_full_array = 0;

for i = 2:length(t_p_full_array_calibrated)   % calculating linear and angular velocities

    d_X_full_array(i) = (X_full_array(i) - X_full_array(i-1)) / (t_p_full_array_calibrated(i) - t_p_full_array_calibrated(i-1));
    d_Y_full_array(i) = (Y_full_array(i) - Y_full_array(i-1)) / (t_p_full_array_calibrated(i) - t_p_full_array_calibrated(i-1));
    d_Z_full_array(i) = (Z_full_array(i) - Z_full_array(i-1)) / (t_p_full_array_calibrated(i) - t_p_full_array_calibrated(i-1));

    d_phi_full_array(i)   = (phi_full_array(i)   - phi_full_array(i-1))   / (t_p_full_array_calibrated(i) - t_p_full_array_calibrated(i-1));
    d_theta_full_array(i) = (theta_full_array(i) - theta_full_array(i-1)) / (t_p_full_array_calibrated(i) - t_p_full_array_calibrated(i-1));
    d_psi_full_array(i)   = (psi_full_array(i)   - psi_full_array(i-1))   / (t_p_full_array_calibrated(i) - t_p_full_array_calibrated(i-1));

end

t_w_full_array_not_calibrated = bagData_wrench.x__time;   % reading the Force/Torque data time array

t_w_full_array_lin = linspace(t_w_full_array_not_calibrated(1), t_w_full_array_not_calibrated(end),...
    length(t_w_full_array_not_calibrated));  % making linearly spaced time stamps to avoid precision errors due to unevenly distributed data points

for i = 1:length(t_w_full_array_lin)

    t_w_full_array_calibrated(i) = t_w_full_array_lin(i) - t_w_full_array_lin(1);   % zero-ing the first time stamp

end

F_x_full_array_raw = interpNaN(t_w_full_array_calibrated, transpose(bagData_wrench.x_bota_serial_wrench_stamped_wrench_force_x));   % reading the contact force in body-fixed frame x-axis
F_y_full_array_raw = interpNaN(t_w_full_array_calibrated, transpose(bagData_wrench.x_bota_serial_wrench_stamped_wrench_force_y));   % reading the contact force in body-fixed frame y-axis
F_z_full_array_raw = interpNaN(t_w_full_array_calibrated, transpose(bagData_wrench.x_bota_serial_wrench_stamped_wrench_force_z));   % reading the contact force in body-fixed frame z-axis

Tau_x_full_array_raw = interpNaN(t_w_full_array_calibrated, transpose(bagData_wrench.x_bota_serial_wrench_stamped_wrench_torque_x));   % reading the contact torque about the body-fixed frame x-axis
Tau_y_full_array_raw = interpNaN(t_w_full_array_calibrated, transpose(bagData_wrench.x_bota_serial_wrench_stamped_wrench_torque_y));   % reading the contact torque about the body-fixed frame y-axis
Tau_z_full_array_raw = interpNaN(t_w_full_array_calibrated, transpose(bagData_wrench.x_bota_serial_wrench_stamped_wrench_torque_z));   % reading the contact torque about the body-fixed frame z-axis

fs = 100;   % data acquisition frequency [Hz]
cof1 = 12;  % cutt-off frequency [Hz]
cof2 = 15;  % cutt-off frequency [Hz]

F_x_full_array = lowpass(F_x_full_array_raw, cof1, fs);   % filtering the F/T sensor x-axis force data
F_y_full_array = lowpass(F_y_full_array_raw, cof1, fs);   % filtering the F/T sensor y-axis force data
F_z_full_array = lowpass(F_z_full_array_raw, cof1, fs);   % filtering the F/T sensor z-axis force data

Tau_x_full_array = lowpass(Tau_x_full_array_raw, cof2, fs);   % filtering the F/T sensor x-axis torque data
Tau_y_full_array = lowpass(Tau_y_full_array_raw, cof2, fs);   % filtering the F/T sensor y-axis torque data
Tau_z_full_array = lowpass(Tau_z_full_array_raw, cof2, fs);   % filtering the F/T sensor z-axis torque data

%%% isolating the collision

i_cp = 1159;   % # of the position   rosbag time-stamp at which the collision happens
i_cw = 969;    % # of the F/T sensor rosbag time-stamp at which the collision happens

t0f = t_p_full_array_calibrated(i_cp-20:i_cp+20) - t_p_full_array_calibrated(i_cp-20);   % isolating and zero-ing the collision time series

X = X_full_array(i_cp-20:i_cp+20);   % isolating the drone X data during the collision
Y = Y_full_array(i_cp-20:i_cp+20);   % isolating the drone Y data during the collision
Z = Z_full_array(i_cp-20:i_cp+20);   % isolating the drone Z data during the collision

d_X = d_X_full_array(i_cp-20:i_cp+20);   % isolating the drone d_X during the collision
d_Y = d_Y_full_array(i_cp-20:i_cp+20);   % isolating the drone d_Y during the collision
d_Z = d_Z_full_array(i_cp-20:i_cp+20);   % isolating the drone d_Z during the collision

phi   = phi_full_array(i_cp-20:i_cp+20);     % isolating the drone roll  angle data during the collision
theta = theta_full_array(i_cp-20:i_cp+20);   % isolating the drone pitch angle data during the collision
psi   = psi_full_array(i_cp-20:i_cp+20);     % isolating the drone yaw   angle data during the collision

d_phi   = d_phi_full_array(i_cp-20:i_cp+20);     % isolating the drone angular velocity d_phi   during the collision
d_theta = d_theta_full_array(i_cp-20:i_cp+20);   % isolating the drone angular velocity d_theta during the collision
d_psi   = d_psi_full_array(i_cp-20:i_cp+20);     % isolating the drone angular velocity d_psi during the collision

F_x = F_x_full_array(i_cw-20:i_cw+20) - F_x_full_array(i_cw-20);   % isolating and zero-ing the contact force along the body-frame x-axis during the collision
F_y = F_y_full_array(i_cw-20:i_cw+20) - F_y_full_array(i_cw-20);   % isolating and zero-ing the contact force along the body-frame y-axis during the collision
F_z = F_z_full_array(i_cw-20:i_cw+20) - F_z_full_array(i_cw-20);   % isolating and zero-ing the contact force along the body-frame z-axis during the collision

%%%
F_xy_raw = F_x_full_array_raw(i_cw-20:i_cw+20) - F_x_full_array_raw(i_cw-20);  % calculating and zeroing the raw normal force in the diraction of contact
F_xy_filtered = sqrt(F_x.^2 + F_y.^2);                                         % calculating the filtered normal force in the diraction of contact
%%%

Tau_x = Tau_x_full_array(i_cw-20:i_cw+20) - Tau_x_full_array(i_cw-20);   % isolating and zero-ing the contact torque about the body-frame x-axis during the collision
Tau_y = Tau_y_full_array(i_cw-20:i_cw+20) - Tau_y_full_array(i_cw-20);   % isolating and zero-ing the contact torque about the body-frame y-axis during the collision
Tau_z = Tau_z_full_array(i_cw-20:i_cw+20) - Tau_z_full_array(i_cw-20);   % isolating and zero-ing the contact torque about the body-frame z-axis during the collision

%%%
Tau_z_raw = Tau_z_full_array_raw(i_cw-20:i_cw+20) - Tau_z_full_array_raw(i_cw-20);
Tau_z_filtered = Tau_z;
%%%

% assigning the initial conditions (generalized coordinates are defined as x = [phi; theta; psi; Z; X; Y; d_phi; d_theta; d_psi; dZ; dX; dY].)

x0(:,1) = [phi(1); -theta(1); -psi(1); -Z(1); X(1); Y(1); d_phi(1); -d_theta(1); -d_psi(1); -d_Z(1); d_X(1); d_Y(1)];

% initiating the simulation variables

F_n_sim(:,1) = [0;0;0]; F_t_sim(:,1) = [0;0;0;]; Tau_n_sim(:,1) = [0;0;0]; Tau_t_sim(:,1) = [0;0;0];
F_X_sim(1) = 0; F_Y_sim(1) = 0; F_Z_sim(1) = 0; Tau_X_sim(1) = 0; Tau_Y_sim(1) = 0; Tau_Z_sim(1) = 0;

% assigning the different coefficient of friction (Mu), damping coefficient
% (B), and srtiffness coefficient (K) value

Mu = 0.2;    % 0.05
B  = 40;      % 40
K  = 2300;    % 2300


% defining the ode solution time and state variables

t = []; x = [];

for ii = 1:(length(t0f)-1)

    u_kbf(:,ii) = [0; 0; 0; 0];   % system input assignment

    [F,q] = lcs_gen_kbf(x0(:,ii),u_kbf(:,ii),Mu,B,K);   % constraint matrix derivation (Complementarity Conditions)

    [w, lambda(:,ii)] = LCPSolve(F,q);    % LCP slution to obtain contact forces "lambda" (w is a slack variable vector)
    
    [F_n_sim(:,ii+1), F_t_sim(:,ii+1), Tau_n_sim(:,ii+1), Tau_t_sim(:,ii+1)] = ...   % extracting the contact forces from the LCP constraint solutions
        Xquad_dyn_FT(x0(:,ii), u_kbf(:,ii), lambda(:,ii));

    F_X_sim(ii+1) = F_n_sim(1,ii+1) + F_t_sim(1,ii+1);
    F_Y_sim(ii+1) = F_n_sim(2,ii+1) + F_t_sim(2,ii+1);
    F_Z_sim(ii+1) = F_n_sim(3,ii+1) + F_t_sim(3,ii+1);

    Tau_X_sim(ii+1) = Tau_n_sim(1,ii+1) + Tau_t_sim(1,ii+1);
    Tau_Y_sim(ii+1) = Tau_n_sim(2,ii+1) + Tau_t_sim(2,ii+1);
    Tau_Z_sim(ii+1) = Tau_n_sim(3,ii+1) + Tau_t_sim(3,ii+1);

    t_span = [t0f(ii) t0f(ii+1)];         % time span for ode input

    [t_period, x_period] = ode15s(@(t,y) Xquad_dyn_ode(t,y,u_kbf(:,ii),lambda(:,ii)), t_span, x0(:,ii));   % solving the system dynamics using the stiff ODE solver: ode15s

    t_period = transpose(t_period);      % output time vector transpose
    x_period = transpose(x_period);      % output state vector transpose

    x0(:,ii+1) = x_period(:,end);        % updating the initial conditions

    t = [t, t_period];                   % adding the solved time period to the time vector
    x = [x, x_period];                   % adding the state solution in the time period to the total states matrix

end

% extracting the simulation points

phi_sim   = x0(1,:);
theta_sim = x0(2,:);
psi_sim   = x0(3,:);
Z_sim = x0(4,:);
X_sim = x0(5,:);
Y_sim = x0(6,:);
d_phi_sim   = x0(7,:);
d_theta_sim = x0(8,:);
d_psi_sim   = x0(9,:);
d_Z_sim = x0(10,:);
d_X_sim = x0(11,:);
d_Y_sim = x0(12,:);

% error calculation between the simulation and experiment
error_dX = mean(abs(d_X_sim - d_X));
error_dY = mean(abs(d_Y_sim - d_Y));
error_Fn = abs(max(abs(F_X_sim)) - max(abs(F_xy_filtered)));
error_Tau_Z = abs(max(abs(Tau_Z_sim)) - max(abs(Tau_z_filtered)));

% error_dX_rel = mean((abs(abs(d_X_sim) - abs(d_X))) ./ max(d_X)) * 100;
% error_dY_rel = mean((abs(d_Y_sim - d_Y)) ./ abs(d_Y)) * 100;
% error_Fn_rel = mean((abs(abs(F_X_sim) - F_xy_filtered)) ./ max(F_xy_filtered)) * 100;
% error_Tau_Z_rel = mean((abs(Tau_Z_sim - Tau_z_filtered)) ./ max(Tau_z_filtered)) * 100;

fprintf('The Error for:\n')
fprintf('Collision Velocity in X-direction (Average) = %4.2f [m.s^-1]\n', error_dX)
fprintf('Collision Velocity in Y-direction (Average) = %4.2f [m.s^-1]\n', error_dY)
fprintf('Normal collision/contact force (Max)= %4.2f [N]\n', error_Fn)
fprintf('Contact torque about world Z-axis (Max) = %4.2f [N.m])\n', error_Tau_Z)


% % calculating the simulation error for K(k), B(j), and Mu(i) 
% 
% w_XY = 100.0; w_dXdY = 10.0; w_F_XY = 1.0; w_Tau_Z = 10.0;   % definfing the optimization cumulative error weights
% 
% N = length(t0f);   % array length
% 
% error_XY(k,j,i) = mean(sqrt((abs(X_sim)-abs(X)).^2 + (abs(Y_sim)-abs(Y)).^2));             % position error [m]
% 
% error_dXdY(k,j,i) = mean(sqrt((abs(d_X_sim)-abs(d_X)).^2 + (abs(d_Y_sim)-abs(d_Y)).^2));   % velocity error [m.s^-1]
% 
% error_F_XY(k,j,i) = mean(abs(abs(F_X_sim) - sqrt(F_x.^2+F_y.^2)));                         % contact normal force error [N]
% 
% error_Tau_Z(k,j,i) = mean(abs(abs(Tau_Z_sim) - abs(Tau_z)));                                    % contact torque about world Z-axis error [N.m]
% 
% error_cum(k,j,i) = (w_XY * error_XY(k,j,i) + w_dXdY * error_dXdY(k,j,i) + ...
%     w_F_XY * error_F_XY(k,j,i) + w_Tau_Z * error_Tau_Z(k,j,i)) / (w_XY + w_dXdY + w_F_XY + w_Tau_Z);   % cumulative weighted error

f1 = figure(1);
plot(t0f, F_xy_raw, 'Color', "#77AC30", "LineStyle", "--","LineWidth", 3)
hold on
plot(t0f, F_xy_filtered, 'Color', "#0072BD", "LineStyle", ":","LineWidth", 4.5, "Marker", "*")
plot(t0f, abs(F_X_sim), 'Color', "#901340", "LineStyle", "-","LineWidth", 6)
ax1 = gca;
ax1.FontSize = 30;
ax1.FontWeight = 'bold';
xlim([0 0.35])
ylim([-10 +50])
% xlabel('\textbf{Time [s]}', 'Interpreter', 'latex', 'FontSize', 40, 'FontWeight', 'bold')
% ylabel({'\textbf{Normal Contact}', '\textbf{Force ${\lambda}_{n}$ [N]}'}, 'Interpreter', 'latex', 'FontSize', 40, 'FontWeight', 'bold')
% legend('Experiment (raw)', 'Experimnet (filtered)', 'Simulation', 'FontSize', 20)
grid on
f1.Units = "inches";
f1.Position = [2, 1.25, 13, 8];

f2 = figure(2);
plot(t0f, -Tau_z_raw, 'Color', "#77AC30", "LineStyle", "--","LineWidth", 2)
hold on
plot(t0f, -Tau_z, 'Color', "#0072BD", "LineStyle", ":","LineWidth", 4.5, "Marker", "*")
plot(t0f, Tau_Z_sim, 'Color', "#901340", "LineStyle", "-","LineWidth", 6)
ax2 = gca;
ax2.FontSize = 30;
ax2.FontWeight = 'bold';
xlim([0 0.35])
ylim([-4 +4])
% xlabel('\textbf{Time [s]}', 'Interpreter', 'latex', 'FontSize', 40, 'FontWeight', 'bold')
% ylabel({'\textbf{Contact Torque}', '\textbf{${\tau}_{Z}$ [N.m]}'}, 'Interpreter', 'latex', 'FontSize', 40, 'FontWeight', 'bold')
% legend('Experiment (raw)', 'Experimnet (filtered)', 'Simulation', 'FontSize', 20)
grid on
f2.Units = "inches";
f2.Position = [1.5, 1, 13, 8];

f3 = figure(3);
plot(t0f, d_X, 'Color', "#0072BD", "LineStyle", ":","LineWidth", 4.5, "Marker", "*")
hold on
plot(t, x(11,:), 'Color', "#901340", "LineStyle", "-","LineWidth", 6)
ax3 = gca;
ax3.FontSize = 30;
ax3.FontWeight = 'bold';
xlim([0 0.35])
ylim([-1.5 +1.5])
% xlabel('\textbf{Time [s]}', 'Interpreter', 'latex', 'FontSize', 40, 'FontWeight', 'bold')
% ylabel({'\textbf{Velocity}', ' \textbf{$v_{X}$ $[m.s^{-1}]$}'}, 'Interpreter', 'latex', 'FontSize', 40, 'FontWeight', 'bold')
% legend('Experiment', 'Simulation', 'FontSize', 20)
grid on
f3.Units = "inches";
f3.Position = [1, 0.75, 13, 8];

f4 = figure(4);
plot(t0f, d_Y, 'Color', "#0072BD", "LineStyle", ":","LineWidth", 4.5, "Marker", "*")
hold on
plot(t, x(12,:), 'Color', "#901340", "LineStyle", "-","LineWidth", 6)
ax4 = gca;
ax4.FontSize = 30;
ax4.FontWeight = 'bold';
xlim([0 0.35])
ylim([-1.5 +1.5])
% xlabel('\textbf{Time [s]}', 'Interpreter', 'latex', 'FontSize', 40, 'FontWeight', 'bold')
% ylabel({'\textbf{Velocity}', ' \textbf{$v_{Y}$ $[m.s^{-1}]$}'}, 'Interpreter', 'latex', 'FontSize', 40, 'FontWeight', 'bold')
% legend('Experiment', 'Simulation', 'FontSize', 20)
grid on
f4.Units = "inches";
f4.Position = [0.5, 0.5, 13, 8];

