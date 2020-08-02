bag=rosbag('2020-08-01-08-29-54.bag');
local_pos=select(bag,'Topic','/mavros/local_position/pose');
local_pos_message=readMessages(local_pos,'DataFormat','struct');
pointx = cellfun(@(m)double(m.Pose.Orientation.X),local_pos_message);
pointy = cellfun(@(m)double(m.Pose.Orientation.Y),local_pos_message);
pointz = cellfun(@(m)double(m.Pose.Orientation.Z),local_pos_message);
pointw = cellfun(@(m)double(m.Pose.Orientation.W),local_pos_message);
q(:,1) = pointw;
q(:,2) = pointx;
q(:,3) = pointy;
q(:,4) = pointz;
[yaw,pitch,roll]=quat2angle(q,'ZYX');


Sec = cellfun(@(m)double(m.Header.Stamp.Sec),local_pos_message);
nSec = cellfun(@(m)double(m.Header.Stamp.Nsec),local_pos_message);
tmp_size = size(Sec);
sec_header = Sec(1);
nsec_header = nSec(1);
for i=1:tmp_size(1)
    Sec(i)=Sec(i)-sec_header;
    nSec(i)=nSec(i)-nsec_header;
    time(i)=Sec(i)+nSec(i)*10^(-9);
end;   


plot(time,rad2deg(roll),'r');
hold on;
plot(time,rad2deg(pitch),'b');
hold on;
plot(time,rad2deg(yaw),'g');
grid on;
    
%% above all show the eular angle 
%% next show the position   
%% the data came from the PX4 estermator (EKF2)
position_x = cellfun(@(m)double(m.Pose.Position.X),local_pos_message);
position_y = cellfun(@(m)double(m.Pose.Position.Y),local_pos_message);
position_z = cellfun(@(m)double(m.Pose.Position.Z),local_pos_message);

figure(2)
subplot(311)
plot(time,position_x);grid on;
subplot(312)
plot(time,position_y);grid on;
subplot(313)
plot(time,position_z);grid on;

figure(3)
plot3(position_x,position_y,position_z);grid on;

%% the next section attach the message from the optitrack
vision_pos = select(bag,'Topic','/mavros/vision_pose/pose');
vision_pos_message = readMessages(vision_pos,'DataFormat','struct');
vision_pos_x = cellfun(@(m)double(m.Pose.Position.X),vision_pos_message);
vision_pos_y = cellfun(@(m)double(m.Pose.Position.Y),vision_pos_message);
vision_pos_z = cellfun(@(m)double(m.Pose.Position.Z),vision_pos_message);
figure(4)
plot3(vision_pos_x,vision_pos_y,vision_pos_z);

optitrack_Sec = cellfun(@(m)double(m.Header.Stamp.Sec),vision_pos_message);
optitrack_nSec = cellfun(@(m)double(m.Header.Stamp.Nsec),vision_pos_message);
tmp_size_optitrack = size(optitrack_Sec);
optitrack_sec_header = optitrack_Sec(1);
optitrack_nsec_header = optitrack_nSec(1);
for i=1:tmp_size_optitrack(1)
    optitrack_Sec(i)=optitrack_Sec(i)-optitrack_sec_header;
    optitrack_nSec(i)=optitrack_nSec(i)-optitrack_nsec_header;
    optitrack_time(i)=optitrack_Sec(i)+optitrack_nSec(i)*10^(-9);
end;   

%% compare the data from the optitrack and EKF2(PX4)
plot(time,position_x,'r');hold on;
plot(optitrack_time,vision_pos_x,'b');
plot3(vision_pos_x,vision_pos_y,vision_pos_z);

%% the next attach the velocity of the drone in the topic /mavros/local_position/velocity_local
velocity_local = select(bag,'Topic','/mavros/local_position/velocity_local');
velocity_local_message = readMessages(velocity_local,'DataFormat','struct');
velocity_local_vx = cellfun(@(m)double(m.Twist.Linear.X),velocity_local_message);
velocity_local_vy = cellfun(@(m)double(m.Twist.Linear.Y),velocity_local_message);
velocity_local_vz = cellfun(@(m)double(m.Twist.Linear.Z),veloctiy_local_message);
velocity_local_Sec = cellfun(@(m)double(m.Header.Stamp.Sec),velocity_local_message);
velocity_local_Nsec = cellfun(@(m)double(m.Header.Stamp.Nsec),velocity_local_message);
tmp_size_velocity_local_time = size(velocity_local_Sec);
velocity_local_sec_header = velocity_local_Sec(1);
velocity_local_nsec_header = velocity_local_Nsec(1);
for i=1:tmp_size_velocity_local_time(1)
    velocity_local_Sec(i)=velocity_local_Sec(i)-velocity_local_sec_header;
    velocity_local_Nsec(i)=velocity_local_Nsec(i) - velocity_local_nsec_header;
    velocity_local_time(i)=velocity_local_Sec(i)+velocity_local_Nsec(i)*10^(-9);
end;
figure('name','velocity_local');
subplot(311)
plot(velocity_local_time,velocity_local_vx);
subplot(312)
plot(velocity_local_time,velocity_local_vy);
subplot(313)
plot(velocity_local_time,velocity_local_vz);

