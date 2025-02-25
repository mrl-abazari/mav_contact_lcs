%%%   this is an Amirali Abazari develped code   %%% 2024.09.22

%%%   this script is written to read the ROS2 bag .csv files and the ROS2 messages
%%%   inside them and plot the required ones   %%%


clc; clear all; close all;


bagData_w = readtable("09_wrench.csv");

t_stamp_full_w = bagData_w.x__time;

t_stamp_lin_w = linspace(t_stamp_full_w(1),t_stamp_full_w(end),length(t_stamp_full_w));

for i = 1:length(t_stamp_full_w)
    t_w(i) = (t_stamp_lin_w(i) - t_stamp_lin_w(1));
end

F_X = interpNaN(t_w, transpose(bagData_w.x_bota_serial_wrench_stamped_wrench_force_x));
F_Y = interpNaN(t_w, transpose(bagData_w.x_bota_serial_wrench_stamped_wrench_force_y));
F_Z = interpNaN(t_w, transpose(bagData_w.x_bota_serial_wrench_stamped_wrench_force_z));

Tau_X = interpNaN(t_w, transpose(bagData_w.x_bota_serial_wrench_stamped_wrench_torque_x));
Tau_Y = interpNaN(t_w, transpose(bagData_w.x_bota_serial_wrench_stamped_wrench_torque_y));
Tau_Z = interpNaN(t_w, transpose(bagData_w.x_bota_serial_wrench_stamped_wrench_torque_z));

fs = 105;
fs_min = 12;

F_X_lpf = lowpass(F_X,fs_min,fs);
F_Y_lpf = lowpass(F_Y,fs_min,fs);
F_Z_lpf = lowpass(F_Z,fs_min,fs);

Tau_X_lpf = lowpass(Tau_X,fs_min,fs);
Tau_Y_lpf = lowpass(Tau_Y,fs_min,fs);
Tau_Z_lpf = lowpass(Tau_Z,fs_min,fs);


figure(1)
plot(t_w,F_X,'b',"LineWidth",2)
hold on
plot(t_w,F_Y,'r',"LineWidth",2)
plot(t_w,F_Z,'g',"LineWidth",2)
ax1 = gca;
ax1.FontSize = 16;
ax1.FontWeight = 'bold';
xlabel('Time [s]')
ylabel('Force [N]')
legend('F_{X}','F_{Y}','F_{Z}')
grid on


figure(2)
plot(t_w,Tau_X,'b',"LineWidth",2)
hold on
plot(t_w,Tau_Y,'r',"LineWidth",2)
plot(t_w,Tau_Z,'g',"LineWidth",2)
ax2 = gca;
ax2.FontSize = 16;
ax2.FontWeight = 'bold';
xlabel('Time [s]')
ylabel('Torque [N.m]')
legend('$\tau_{X}$','$\tau_{Y}$','$\tau_{Z}$','Interpreter','latex')
grid on


figure(3)

tiledlayout(2,3);

nexttile
plot(t_w,F_X,'r-.',"LineWidth",1)
hold on
plot(t_w,F_X_lpf,'b',"LineWidth",2)
ax11 = gca;
ax11.FontSize = 16;
ax11.FontWeight = 'bold';
xlabel('Time [s]')
ylabel('Force [N]')
legend('F_{X} Raw','F_{X} Filtered')
grid on

nexttile
plot(t_w,F_Y,'m-.',"LineWidth",1)
hold on
plot(t_w,F_Y_lpf,'g',"LineWidth",2)
ax12 = gca;
ax12.FontSize = 16;
ax12.FontWeight = 'bold';
xlabel('Time [s]')
ylabel('Force [N]')
legend('F_{Y} Raw','F_{Y} Filtered')
grid on

nexttile
plot(t_w,F_Z,'y-.',"LineWidth",1)
hold on
plot(t_w,F_Z_lpf,'c',"LineWidth",2)
ax13 = gca;
ax13.FontSize = 16;
ax13.FontWeight = 'bold';
xlabel('Time [s]')
ylabel('Force [N]')
legend('F_{Z} Raw','F_{Z} Filtered')
grid on

nexttile
plot(t_w,Tau_X,'r-.',"LineWidth",1)
hold on
plot(t_w,Tau_X_lpf,'b',"LineWidth",2)
ax21 = gca;
ax21.FontSize = 16;
ax21.FontWeight = 'bold';
xlabel('Time [s]')
ylabel('Torque [N.m]')
legend('$\tau_{X}$ Raw','$\tau_{X}$ Filtered','Interpreter','latex')
grid on

nexttile
plot(t_w,Tau_Y,'m-.',"LineWidth",1)
hold on
plot(t_w,Tau_Y_lpf,'g',"LineWidth",2)
ax22 = gca;
ax22.FontSize = 16;
ax22.FontWeight = 'bold';
xlabel('Time [s]')
ylabel('Torque [N.m]')
legend('$\tau_{Y}$ Raw','$\tau_{Y}$ Filtered','Interpreter','latex')
grid on

nexttile
plot(t_w,Tau_Z,'y-.',"LineWidth",1)
hold on
plot(t_w,Tau_Z_lpf,'c',"LineWidth",2)
ax23 = gca;
ax23.FontSize = 16;
ax23.FontWeight = 'bold';
xlabel('Time [s]')
ylabel('Torque [N.m]')
legend('$\tau_{Z}$ Raw','$\tau_{Z}$ Filtered','Interpreter','latex')
grid on



bagData_p = readtable("09_pose.csv");

t_stamp_full_p = bagData_p.x__time;

t_stamp_lin_p = linspace(t_stamp_full_p(1),t_stamp_full_p(end),length(t_stamp_full_p));

for i = 1:length(t_stamp_full_p)
    t_p(i) = (t_stamp_lin_p(i) - t_stamp_lin_p(1));
end

X = interpNaN(t_p, transpose(bagData_p.x_vrpn_mocap_RigidBody1_pose_pose_position_x));
Y = interpNaN(t_p, transpose(bagData_p.x_vrpn_mocap_RigidBody1_pose_pose_position_y));
Z = interpNaN(t_p, transpose(bagData_p.x_vrpn_mocap_RigidBody1_pose_pose_position_z));

phi = interpNaN(t_p, transpose(bagData_p.x_vrpn_mocap_RigidBody1_pose_pose_orientation_roll));
theta = interpNaN(t_p, transpose(bagData_p.x_vrpn_mocap_RigidBody1_pose_pose_orientation_pitch));
psi = interpNaN(t_p, transpose(bagData_p.x_vrpn_mocap_RigidBody1_pose_pose_orientation_yaw));

d_X(1) = 0; d_Y(1) = 0; d_Z(1) = 0; d_phi(1) = 0; d_theta(1) = 0; d_psi(1) = 0;

for i = 2:length(t_p)
    d_X(i) = (X(i) - X(i-1)) / (t_p(i) - t_p(i-1));
    d_Y(i) = (Y(i) - Y(i-1)) / (t_p(i) - t_p(i-1));
    d_Z(i) = (Z(i) - Z(i-1)) / (t_p(i) - t_p(i-1));

    d_phi(i) = (phi(i) - phi(i-1)) / (t_p(i) - t_p(i-1));
    d_theta(i) = (theta(i) - theta(i-1)) / (t_p(i) - t_p(i-1));
    d_psi(i) = (psi(i) - psi(i-1)) / (t_p(i) - t_p(i-1));
end

d_X_lpf = lowpass(d_X,fs_min,fs);
d_Y_lpf = lowpass(d_Y,fs_min,fs);
d_Z_lpf = lowpass(d_Z,fs_min,fs);

d_phi_lpf = lowpass(d_phi,fs_min,fs);
d_theta_lpf = lowpass(d_theta,fs_min,fs);
d_psi_lpf = lowpass(d_psi,fs_min,fs);

figure(4)

tiledlayout(2,3);

nexttile
plot(t_p,X,'r-.',"LineWidth",1)
hold on
plot(t_p,d_X,'b',"LineWidth",2)
ax11 = gca;
ax11.FontSize = 16;
ax11.FontWeight = 'bold';
xlabel('Time [s]')
ylabel('')
legend('X [m]','v_{X} [ms^{-1}]')
grid on

nexttile
plot(t_p,Y,'m-.',"LineWidth",1)
hold on
plot(t_p,d_Y,'g',"LineWidth",2)
ax12 = gca;
ax12.FontSize = 16;
ax12.FontWeight = 'bold';
xlabel('Time [s]')
ylabel('')
legend('Y [m]','v_{Y} [ms^{-1}]')
grid on

nexttile
plot(t_p,Z,'y-.',"LineWidth",1)
hold on
plot(t_p,d_Z,'c',"LineWidth",2)
ax13 = gca;
ax13.FontSize = 16;
ax13.FontWeight = 'bold';
xlabel('Time [s]')
ylabel('')
legend('Z [m]','v_{Z} [ms^{-1}]')
grid on

nexttile
plot(t_p,phi,'r-.',"LineWidth",1)
hold on
plot(t_p,d_phi,'b',"LineWidth",2)
ax11 = gca;
ax11.FontSize = 16;
ax11.FontWeight = 'bold';
xlabel('Time [s]')
ylabel('')
legend('$\phi$ [rad]','$\dot{\phi} [rad.s^{-1}$]','interpreter','latex')
grid on

nexttile
plot(t_p,theta,'m-.',"LineWidth",1)
hold on
plot(t_p,d_theta,'g',"LineWidth",2)
ax12 = gca;
ax12.FontSize = 16;
ax12.FontWeight = 'bold';
xlabel('Time [s]')
ylabel('')
legend('$\theta$ [rad]','$\dot{\theta} [rad.s^{-1}$]','interpreter','latex')
grid on

nexttile
plot(t_p,psi,'y-.',"LineWidth",1)
hold on
plot(t_p,d_psi,'c',"LineWidth",2)
ax12 = gca;
ax12.FontSize = 16;
ax12.FontWeight = 'bold';
xlabel('Time [s]')
ylabel('')
legend('$\psi$ [rad]','$\dot{\psi} [rad.s^{-1}$]','interpreter','latex')
grid on

