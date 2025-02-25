clc; clear all; close all;

folderPath = fullfile(pwd,"rosbag2_2024_09_20-14_46_46");
bag = ros2bag(folderPath);
bagInfo = ros2("bag","info", folderPath)

msgs = readMessages(bag);

% bagSel = select(bag,"Topic","/fmu/vehicle_odometry/out");
bagSel_1 = select(bag, "Topic", "/RigidBody1/pose");
bagSel_2 = select(bag, "Topic", "/bota_serial/wrench_stamped");

msgsFiltered_pose = readMessages(bagSel_1);
msgsFiltered_wrench = readMessages(bagSel_2);

[nc1, nr1] = size(msgsFiltered_pose);
[nc2, nr2] = size(msgsFiltered_wrench);


dt = 1 / 120;
t_span_1 = 0:dt:(nc1-1)*dt;
t_span_2 = 0:dt:(nc1-2)*dt;

n1 = 0;
n2 = 0;

for i = 1:nc1
    bagData_pose(i) = msgsFiltered_pose{i,1};
    
    X(i) = bagData_pose(i).pose.position.x;
    Y(i) = bagData_pose(i).pose.position.y;
    Z(i) = bagData_pose(i).pose.position.z;
    
    quat(:,i) = [bagData_pose(i).pose.orientation.x; bagData_pose(i).pose.orientation.y; bagData_pose(i).pose.orientation.z; bagData_pose(i).pose.orientation.w];
    eul(:,i) = transpose(quat2eul(transpose(quat(:,i))));

    phi(i) = eul(1,i);
    theta(i) = eul(2,i);
    psi(i) = eul(3,1);
    
    t1_stamp_Nsec(i) = bagData_pose(i).header.stamp.nanosec;
    if i > 1
        if t1_stamp_Nsec(i) - t1_stamp_Nsec(i-1) <= 0
            n1 = n1+1;
            t1(i) = double(n1) + double(t1_stamp_Nsec(i)) / double(10^9);
        else
            t1(i) = double(n1) + double(t1_stamp_Nsec(i)) / double(10^9);
        end
    else
        t1(i) = double(n1) + double(t1_stamp_Nsec(i)) / double(10^9);
    end
end

v_X(1) = 0;
v_Y(1) = 0;
v_Z(1) = 0; 
for j = 2:nc1
    v_X(j) = (X(j) - X(j-1)) / dt;
    v_Y(j) = (Y(j) - Y(j-1)) / dt;
    v_Z(j) = (Z(j) - Z(j-1)) / dt;
end
fs = 100;
v_X_lpf = lowpass(v_X, 10, fs);
v_Y_lpf = lowpass(v_Y, 10, fs);
v_Z_lpf = lowpass(v_Z, 10, fs);


for k = 1:nc2
    bagData_wrench(k) = msgsFiltered_wrench{k,1};

    FX(k) = bagData_wrench(k).wrench.force.x;
    FY(k) = bagData_wrench(k).wrench.force.y;
    FZ(k) = bagData_wrench(k).wrench.force.z;

    TauX(k) = bagData_wrench(k).wrench.torque.x;
    TauY(k) = bagData_wrench(k).wrench.torque.y;
    TauZ(k) = bagData_wrench(k).wrench.torque.z;

    t2_stamp_Nsec(k) = bagData_wrench(k).header.stamp.nanosec;
    if k > 1
        if t2_stamp_Nsec(k) - t2_stamp_Nsec(k-1) <= 0
            n2 = n2+1;
            t2(k) = double(n2) + double(t2_stamp_Nsec(k)) / double(10^9);
        else
            t2(k) = double(n2) + double(t2_stamp_Nsec(k)) / double(10^9);
        end
    else
        t2(k) = double(n2) + double(t2_stamp_Nsec(k)) / double(10^9);
    end
end

FX_lpf = lowpass(FX, 10, fs);
FY_lpf = lowpass(FY, 10, fs);
FZ_lpf = lowpass(FZ, 10, fs);

TauX_lpf = lowpass(TauX, 10, fs);
TauY_lpf = lowpass(TauY, 10, fs);
TauZ_lpf = lowpass(TauZ, 10, fs);



figure(1)

tiledlayout(1,3)

nexttile
plot(t1, X, LineWidth=1.5)
ax1 = gca;
ax1.FontSize = 14;
ax1.FontWeight = 'bold';
hold on
plot(t1, v_X, LineWidth=1.5)
xlabel("Time [s]")
legend("X [m]","v_{X} [ms^{-1}]")
grid on

nexttile
plot(t1, Y, LineWidth=1.5)
ax2 = gca;
ax2.FontSize = 14;
ax2.FontWeight = 'bold';
hold on
plot(t1, v_Y, LineWidth=1.5)
xlabel("Time [s]")
legend("Y [m]","v_{Y} [ms^{-1}]")
grid on

nexttile
plot(t1, Z, LineWidth=1.5)
ax1 = gca;
ax1.FontSize = 14;
ax1.FontWeight = 'bold';
hold on
plot(t1, v_Z, LineWidth=1.5)
xlabel("Time [s]")
legend("Z [m]","v_{Z} [ms^{-1}]")
grid on



figure(2)

tiledlayout(2,3)

nexttile
plot(t2, FX, LineWidth=1.5)
ax11 = gca;
ax11.FontSize = 14;
ax11.FontWeight = 'bold';
hold on
plot(t2, FX_lpf, LineWidth=1.5)
xlabel("Time [s]")
legend("F_{X} [N]","F_{X} lpf [N]")
grid on

nexttile
plot(t2, FY, LineWidth=1.5)
ax12 = gca;
ax12.FontSize = 14;
ax12.FontWeight = 'bold';
hold on
plot(t2, FY_lpf, LineWidth=1.5)
xlabel("Time [s]")
legend("F_{Y} [N]","F_{Y} lpf [N]")
grid on

nexttile
plot(t2, FZ, LineWidth=1.5)
ax13 = gca;
ax13.FontSize = 14;
ax13.FontWeight = 'bold';
hold on
plot(t2, FZ_lpf, LineWidth=1.5)
xlabel("Time [s]")
legend("F_{Z} [N]","F_{Z} lpf [N]")
grid on

nexttile
plot(t2, TauX, LineWidth=1.5)
ax21 = gca;
ax21.FontSize = 14;
ax21.FontWeight = 'bold';
hold on
plot(t2, TauX_lpf, LineWidth=1.5)
xlabel("Time [s]")
legend("$\tau_{X}$ [N.m]","$\tau_{X}$ lpf [N.m]", 'Interpreter', 'latex')
grid on

nexttile
plot(t2, TauY, LineWidth=1.5)
ax22 = gca;
ax22.FontSize = 14;
ax22.FontWeight = 'bold';
hold on
plot(t2, TauY_lpf, LineWidth=1.5)
xlabel("Time [s]")
legend("$\tau_{Y}$ [N.m]","$\tau_{Y}$ lpf [N.m]", 'Interpreter', 'latex')
grid on

nexttile
plot(t2, TauZ, LineWidth=1.5)
ax23 = gca;
ax23.FontSize = 14;
ax23.FontWeight = 'bold';
hold on
plot(t2, TauZ_lpf, LineWidth=1.5)
xlabel("Time [s]")
legend("$\tau_{Z}$ [N.m]","$\tau_{Z}$ lpf [N.m]", 'Interpreter', 'latex')
grid on

