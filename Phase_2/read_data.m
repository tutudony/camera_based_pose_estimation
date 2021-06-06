close all
clear all
clc
%%load data from file and initilize
load('./data/studentdata1.mat');

%pose = struct('t',[],'p',[]);
time_vicon=time;
time_data=cat(1,data.t);
vel_vicon = vicon(7:9,:);
omg_vicon = vicon(10:12,:);
rpy_vicon = vicon(4:6,:);
omg_imu = [];
acc_imu = [];
dimension = size(time_data,1);
for i=1:dimension
    omg_imu(:,i) = data(i).omg;
    acc_imu(:,i) = data(i).acc;
end
%%
[pos_arr,vicon_arr,rpy_arr,vicon_arr_ang,time_arr,vel_arr] = on_board(data,vicon,time_vicon,time_data,omg_imu,acc_imu);
%%
% size(pos_arr)
size(vel_arr)
%%

% output phase 1
figure('Name','roll');
plot(time_arr,vicon_arr_ang(:,1)) 
hold on 
plot(time_arr,rpy_arr(:,2))
hold off

figure('Name','pitch');
plot(time_arr,vicon_arr_ang(:,2)) 
hold on 
plot(time_arr,rpy_arr(:,3))
hold off

figure('Name','Yaw');
plot(time_arr,vicon_arr_ang(:,3)) 
hold on 
plot(time_arr,rpy_arr(:,4))
hold off

figure('Name','X');
plot(time_arr,vicon_arr(:,1)) 
hold on 
plot(time_arr,pos_arr(:,1))
hold off

figure('Name','Y');
plot(time_arr,vicon_arr(:,2)) 
hold on 
plot(time_arr,pos_arr(:,2))
hold off

figure('Name','Z');
plot(time_arr,vicon_arr(:,3)) 
hold on 
plot(time_arr,pos_arr(:,3))
hold off

%%

% output phase 2
figure('Name','Vx');
plot(time_arr,vicon_arr(:,7)) 
hold on 
plot(time_arr,vel_arr(:,1))
hold off

figure('Name','Vy');
plot(time_arr,vicon_arr(:,8)) 
hold on 
plot(time_arr,vel_arr(:,2))
hold off

figure('Name','Vz');
plot(time_arr,vicon_arr(:,9)) 
hold on 
plot(time_arr,vel_arr(:,3))
hold off

figure('Name','Wx');
plot(time_arr,vicon_arr(:,10)) 
hold on 
plot(time_arr,vel_arr(:,4))
hold off

figure('Name','Wy');
plot(time_arr,vicon_arr(:,11)) 
hold on     
plot(time_arr,vel_arr(:,5))
hold off

figure('Name','Wz');
plot(time_arr,vicon_arr(:,12)) 
hold on 
plot(time_arr,vel_arr(:,6))
hold off
