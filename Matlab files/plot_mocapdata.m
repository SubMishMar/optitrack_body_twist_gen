clc;
clear all;



Data_mocap = importdata('/bagfiles/Trial1/trialData_360.txt');
M = Data_mocap.data;
time_mocap = M(:,1) * 10^(-9);

%%
% X Y Z displacement plot
mocap_x = M(:,6);
mocap_y = M(:,7);
mocap_z = M(:,8);

figure; 
hold on;

subplot(3,2,1);
plot(time_mocap, mocap_x, 'r');
xlabel('Time[s] since epoch');
ylabel('Displacement[m] - X');
title('Position X');
hold on;
grid on;
subplot(3,2,3);
plot(time_mocap, mocap_y, 'r');
xlabel('Time[s] since epoch');
ylabel('Displacement[m] - Y');
title('Position Y');
hold on;
grid on;
subplot(3,2,5);
plot(time_mocap, mocap_z, 'r');
xlabel('Time[s] since epoch');
ylabel('Displacement[m] - Z');
title('Position Z');
hold on;
grid on;
%%
% Roll Pitch Yaw plot
mocap_qx = M(:,9);
mocap_qy = M(:,10);
mocap_qz = M(:,11);
mocap_qw = M(:,12);

ex = mocap_qx;
ey = mocap_qy;
ez = mocap_qz;
eta = mocap_qw;
roll = [];
pitch = [];
yaw = [];
R = [];
for i = 1:size(ex,1)
    
    R = [2*(eta(i)^2+ex(i)^2)-1, 2*(ex(i)*ey(i)-eta(i)*ez(i)), 2*(ex(i)*ez(i)+eta(i)*ey(i));
    2*(ex(i)*ey(i)+eta(i)*ez(i)), 2*(eta(i)^2+ey(i)^2)-1, 2*(ey(i)*ez(i)-eta(i)*ex(i));
    2*(ex(i)*ez(i)-eta(i)*ey(i)), 2*(ey(i)*ez(i)+eta(i)*ex(i)), 2*(eta(i)^2+ez(i)^2)-1];
    
    
    roll = [roll; atan2(R(3,2),R(3,3))] * 180/pi;
    pitch = [pitch; atan2(-R(3,1),sqrt(R(3,2)^2+R(3,3)^2))] * 180/pi;
    yaw = [yaw; atan2(R(2,1),R(1,1))] * 180/pi;
    
end


subplot(3,2,2);
plot(time_mocap, roll, 'r');
xlabel('Time[s] since epoch');
ylabel('Roll[deg]');
title('Roll');
hold on;
grid on;
subplot(3,2,4);
plot(time_mocap, pitch, 'r');
xlabel('Time[s] since epoch');
ylabel('Pitch[deg]');
title('Pitch');
hold on;
grid on;
subplot(3,2,6);
plot(time_mocap, yaw, 'r');
xlabel('Time[s] since epoch');
ylabel('Yaw[deg]');
title('Yaw');
hold on;
grid on;


%%
% Linear velocity in X Y Z plot
mocap_twist_linear_x = M(:,49);
mocap_twist_linear_y = M(:,50);
mocap_twist_linear_z = M(:,51);

figure; hold on; grid on;

subplot(3,2,1);
plot(time_mocap, mocap_twist_linear_x, 'r');
xlabel('Time[s] since epoch');
ylabel('Linear Velocity[m/s] - X');
title('Linear Velocity X');
hold on;
grid on;
subplot(3,2,3);
plot(time_mocap, mocap_twist_linear_y, 'r');
xlabel('Time[s] since epoch');
ylabel('Linear Velocity[m/s] - Y');
title('Linear Velocity Y');
hold on;
subplot(3,2,5);
plot(time_mocap, mocap_twist_linear_z, 'r');
xlabel('Time[s] since epoch');
ylabel('Linear Velocity[m/s] - Z');
title('Linear Velocity Z');
hold on;
grid on;
%%
% Angular velocity plot

mocap_twist_angular_x = M(:,52).* (180 / pi);
mocap_twist_angular_y = M(:,53).* (180 / pi);
mocap_twist_angular_z = M(:,54).* (180 / pi);

subplot(3,2,2);
plot(time_mocap, mocap_twist_angular_x, 'r');
xlabel('Time[s] since epoch');
ylabel('Angular Velocity[rad/s] - X');
title('Angular Velocity X');
hold on;
grid on;
subplot(3,2,4);
plot(time_mocap, mocap_twist_angular_y, 'r');
xlabel('Time[s] since epoch');
ylabel('Angular Velocity[rad/s] - Y');
title('Angular Velocity Y');
hold on;
grid on;
subplot(3,2,6);
plot(time_mocap, mocap_twist_angular_z, 'r');
xlabel('Time[s] since epoch');
ylabel('Angular Velocity[rad/s] - Z');
title('Angular Velocity Z');
hold on;
grid on;


