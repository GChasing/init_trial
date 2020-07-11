#include "rl_lib.h"
#include "rl_altitude_control.hpp"

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pos_position = *msg;
    pos_current = ToEigen(local_pos_position.pose.position);
}

void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    local_vel_current = *msg;
    ver_current = ToEigen(local_vel_current.twist.linear);
}

int init_finished(geometry_msgs::PoseStamped pos)
{  
    static uint16_t count = 0;
    float error_z = local_pos_position.pose.position.z - pos.pose.position.z;
    if(fabs(error_z)<0.2)
    {
        count ++;
    }
    else{
        count = 0;
    }
    if(count>50)
    {
        return 1;
    }
    return 0;
}

int main(int argc, char **argv)
{
    double tmp[minimum_snap_Row][minimum_snap_Col];
    double time_tmp[minimum_snap_Row];

    std::ifstream openfile;
    openfile.open("/home/chasing/Documents/minimum_snap/data.txt",std::ios::in);
    if(!openfile.is_open())
    {
        ROS_ERROR("File open failed! Must check it, and repeated it!");
        return 1;
    }
    for(uint8_t i=0;i<minimum_snap_Row;i++){
        for(uint8_t j=0;j<minimum_snap_Col;j++){
            openfile >> tmp[i][j];
            _polyCoeff(i,j) = tmp[i][j];
        }
    }
    for(uint8_t i = 0;i<minimum_snap_Row;i++){
        openfile >> time_tmp[i];
        _polyTime(i) = time_tmp[i];
    }
    openfile.close();
    // visWayPointTraj(_polyCoeff,_polyTime);
    // std::cout<<_polyCoeff<<std::endl<<_polyTime<<std::endl;

    uint8_t code_step = 0;
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Publisher local_attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude",10);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Publisher local_thrust_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpointraw_attitude",10);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,local_pos_cb);
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_local",10,local_vel_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(125.0);  //100hz

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }


    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    Eigen::Vector3d EularAngle_current;
    ros::Time Circle_begin_t;
    uint8_t Circle_T_record = 1;
    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
            else{
                if(init_finished(pose) || code_step){
                    if(Circle_T_record){
                        Circle_begin_t = ros::Time::now();
                        Circle_T_record = 0;
                    }
                    Circle_trajectory(local_pos_position,Circle_begin_t);
                    local_attitude_pub.publish(local_attitude_target);
                    code_step = 1;
                }
                else{
                    code_step = 0;
                }
            }
        }
        if(!code_step){
            local_pos_pub.publish(pose);
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}