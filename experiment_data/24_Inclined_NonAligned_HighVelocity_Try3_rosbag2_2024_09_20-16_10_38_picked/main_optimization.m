%%%   this is an Amirali Abazari developed code   %%%   25 September 2024

%%%   _-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_   %%%

%%%   this script is written to read the collision experiment data, isolate
%%%   the collision point, obtain the I.C.s and feed it to the simulation
%%%   model, run the simulation code changing the stiffness, damping, and
%%%   coefficient of friction iteratively and calculating the error between
%%%   simulation and experiment. The error is then plotted as a surface in
%%%   the k(k), b(j) surface for different u(i) values ((k,j,i) are the 
%%%   number of different stiffness, damping, and coefficient of friction 
%%%   values used tried for optimization)

%% optimization part

%%%   _-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_   %%%

clc; clear all; close all;

%%% reading the experiment rosbag data

bagData_pose   = readtable("24_pose.csv");
bagData_wrench = readtable("24_wrench.csv");

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
cof = 12;   % cutt-off frequencyt [Hz]

F_x_full_array = lowpass(F_x_full_array_raw, cof, fs);   % filtering the F/T sensor x-axis force data
F_y_full_array = lowpass(F_y_full_array_raw, cof, fs);   % filtering the F/T sensor y-axis force data
F_z_full_array = lowpass(F_z_full_array_raw, cof, fs);   % filtering the F/T sensor z-axis force data

Tau_x_full_array = lowpass(Tau_x_full_array_raw, cof, fs);   % filtering the F/T sensor x-axis torque data
Tau_y_full_array = lowpass(Tau_y_full_array_raw, cof, fs);   % filtering the F/T sensor y-axis torque data
Tau_z_full_array = lowpass(Tau_z_full_array_raw, cof, fs);   % filtering the F/T sensor z-axis torque data

%%% isolating the collision

i_cp = 1047;   % # of the position   rosbag time-stamp at which the collision happens
i_cw = 865;    % # of the F/T sensor rosbag time-stamp at which the collision happens

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

Tau_x = Tau_x_full_array(i_cw-20:i_cw+20) - Tau_x_full_array(i_cw-20);   % isolating and zero-ing the contact torque about the body-frame x-axis during the collision
Tau_y = Tau_y_full_array(i_cw-20:i_cw+20) - Tau_y_full_array(i_cw-20);   % isolating and zero-ing the contact torque about the body-frame y-axis during the collision
Tau_z = Tau_z_full_array(i_cw-20:i_cw+20) - Tau_z_full_array(i_cw-20);   % isolating and zero-ing the contact torque about the body-frame z-axis during the collision

% assigning the initial conditions (generalized coordinates are defined as x = [phi; theta; psi; Z; X; Y; d_phi; d_theta; d_psi; dZ; dX; dY].)

x0(:,1) = [phi(1); theta(1); psi(1); Z(1); X(1); Y(1); d_phi(1); d_theta(1); d_psi(1); d_Z(1); d_X(1); d_Y(1)];

% initiating the simulation variables

F_n_sim(:,1) = [0;0;0]; F_t_sim(:,1) = [0;0;0;]; Tau_n_sim(:,1) = [0;0;0]; Tau_t_sim(:,1) = [0;0;0];
F_X_sim(1) = 0; F_Y_sim(1) = 0; F_Z_sim(1) = 0; Tau_X_sim(1) = 0; Tau_Y_sim(1) = 0; Tau_Z_sim(1) = 0;

% assigning the different coefficient of friction (Mu), damping coefficient
% (B), and srtiffness coefficient (K) value

% Mu = 0.05:0.025:0.2;
% B = 10:5:70;
% K = 1500:100:4500;

Mu = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9];
B = [1e-2, 1e-1, 1e0, 1e1, 1e2, 1e3, 1e4, 1e5];
K = [1e-2, 1e-1, 1e0, 1e1, 1e2, 1e3, 1e4, 1e5];

% defining the ode solution time and state variables

t = []; x = [];


for i = 1:length(Mu)

    for j = 1:length(B)

        for k = 1:length(K)

            for ii = 1:(length(t0f)-1)
            
                u_kbf(:,ii) = [0; 0; 0; 0];   % system input assignment
    
                [F,q] = lcs_gen_kbf(x0(:,ii),u_kbf(:,ii),Mu(i),B(j),K(k));   % constraint matrix derivation (Complementarity Conditions)
    
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
            

            % calculating the simulation error for K(k), B(j), and Mu(i) 

            w_XY = 100.0; w_dXdY = 10.0; w_F_XY = 1.0; w_Tau_Z = 10.0;   % definfing the optimization cumulative error weights
            
            N = length(t0f);   % array length

            error_XY(k,j,i) = mean(sqrt((abs(X_sim)-abs(X)).^2 + (abs(Y_sim)-abs(Y)).^2));             % position error [m]

            error_dXdY(k,j,i) = mean(sqrt((abs(d_X_sim)-abs(d_X)).^2 + (abs(d_Y_sim)-abs(d_Y)).^2));   % velocity error [m.s^-1]
            
            error_F_XY(k,j,i) = mean(abs(abs(F_X_sim) - sqrt(F_x.^2+F_y.^2)));                         % contact normal force error [N]

            error_Tau_Z(k,j,i) = mean(abs(abs(Tau_Z_sim) - abs(Tau_z)));                                    % contact torque about world Z-axis error [N.m]

            error_cum(k,j,i) = (w_XY * error_XY(k,j,i) + w_dXdY * error_dXdY(k,j,i) + ...
                w_F_XY * error_F_XY(k,j,i) + w_Tau_Z * error_Tau_Z(k,j,i)) / (w_XY + w_dXdY + w_F_XY + w_Tau_Z);   % cumulative weighted error

            fprintf('For K = %4.2e , B = %4.2e , Mu = %4.3f   :   error_XY = %4.2f , error_dXdY = %4.2f , error_F_XY = %4.2f , error_Tau_Z = %4.2f , error_cum = %4.4f \n', ...
                K(k), B(j), Mu(i), error_XY(k,j,i), error_dXdY(k,j,i), error_F_XY(k,j,i), error_Tau_Z(k,j,i), error_cum(k,j,i));

        end

    end

end


%% plotting

for i = 1:length(Mu)
    
    
    figure(1)
    surf(B, K, error_cum(:,:,i), 'FaceAlpha', 0.5);
    hold on
    ax1 = gca;
    ax1.FontSize = 16;
    ax1.FontWeight = 'bold';
    ax1.XScale = 'log';
    ax1.YScale = 'log';
    xlabel('damping ratio (b)')
    ylabel('Stiffness ratio (k)')
    zlabel('Weighted Error')
    % zlim([0.0 1.0])


end