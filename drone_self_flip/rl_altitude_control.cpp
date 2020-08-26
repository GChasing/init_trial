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

void imu_data_cb(const sensor_msgs::Imu::ConstPtr& msg){
    imu_data = *msg;
    AngleRate_current = ToEigen(imu_data.angular_velocity);
}

int start_position_check(geometry_msgs::PoseStamped pos)
{  
    static uint16_t count = 0;
    float error_z = local_pos_position.pose.position.z - pos.pose.position.z;
    float error_x = local_pos_position.pose.position.x - pos.pose.position.x;
    float error_y = local_pos_position.pose.position.y - pos.pose.position.y;
    if(fabs(error_z)<0.2 && fabs(error_y)<0.2 && fabs(error_x)<0.2)
    {
        count ++;
    }
    else{
        count = 0;
    }
    if(count>150)
    {
        return 1;
    }
    return 0;
}

double tmp[minimum_snap_Row][minimum_snap_Col];
double time_tmp[minimum_snap_Row];
int8_t read_minimum_snap_para(void){
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
    return 0;
}

// void callback(drone_new::dynamic_paramConfig &config, uint32_t level){
//     KR1 = config.KR1;
//     KR2 = config.KR2;
//     KR3 = config.KR3;

//     KOmega1 = config.KOmega1;
//     KOmega2 = config.KOmega2;
//     KOmega3 = config.KOmega3;
// }

#define geometric_control              //if not using geometric control , please cancel this line 
int main(int argc, char **argv)
{
    // read_minimum_snap_para();
    uint8_t code_step = 1;
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
    ros::Subscriber imu_data_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data",10,imu_data_cb);
    // dynamic_reconfigure::Server<drone_new::dynamic_paramConfig> server;
    // dynamic_reconfigure::Server<drone_new::dynamic_paramConfig>::CallbackType f;
    // f = boost::bind(&callback,_1,_2);
    // server.setCallback(f);

    std::string traj_type;
    std::string controller;
    nh.param<std::string>("/type",traj_type,"circle");
    nh.param<std::string>("/controller",controller,"attitude_eth");
    nh.param<double>("/KR1",KR1,10);
    nh.param<double>("/KR2",KR2,10);
    nh.param<double>("/KR3",KR3,10);
    nh.param<double>("/KOmega1",KOmega1,0.5);
    nh.param<double>("/KOmega2",KOmega2,0.5);
    nh.param<double>("/KOmega3",KOmega3,0.5);
    nh.param<double>("/traj_T",traj_T,3.0);   //repeat period
    nh.param<double>("/k_pos1",k_pos1,-8);
    nh.param<double>("/k_pos2",k_pos2,-8);
    nh.param<double>("/k_pos3",k_pos3,-10);
    nh.param<double>("/k_vel1",k_vel1,-1.5);
    nh.param<double>("/k_vel2",k_vel2,-3.3);
    nh.param<double>("/k_vel3",k_vel3,-3.3);
    ros::Rate rate(250.0);  //100hz

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    #ifdef geometric_control
        pose.pose.position.x = 0.75;
        pose.pose.position.y = 0;
        if(controller == "attitude_flip")
            pose.pose.position.z = 1;
        else
            pose.pose.position.z = 1;
        pose.pose.orientation.w = 0;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 1;
    #else
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 1;
    #endif

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    Eigen::Vector3d EularAngle_current;
    ros::Time Circle_begin_t;
    uint8_t Offboard_control_ = 0;
    uint8_t RecoryToInit = 0;
    while(ros::ok()){ 
        if( current_state.mode == "OFFBOARD" && current_state.armed )
        {
            Offboard_control_ = 1;
            if(!start_position_check(pose) && code_step)
            {
                // local_pos_pub.publish(pose);
                // code_step = 0;
                RecoryToInit = 0;
                finish_flip = 0;
                Circle_begin_t = ros::Time::now();
                local_pos_pub.publish(pose);
            }
            else{
                code_step = 0;
                #ifdef geometric_control
                    if(finish_flip)
                    {
                        code_step = 1;
                        Offboard_control_ = 0;
                        pose.pose.position.x = 0.75;
                        pose.pose.position.y = 0;
                        pose.pose.position.z = 1;
                        RecoryToInit = 1;
                    }
                    Circle_trajectory(imu_data,local_pos_position,Circle_begin_t,traj_type,controller);
                    local_attitude_pub.publish(local_attitude_target);
                #else
                    local_pos_pub.publish(pose);
                #endif
            }
        }
        else{
            Circle_begin_t = ros::Time::now();
        }
        if(!Offboard_control_)
        {
            local_pos_pub.publish(pose);
            if(start_position_check(pose))
            {
                Offboard_control_ = 1;
                RecoryToInit = 0;
                finish_flip = 0;
                Circle_begin_t = ros::Time::now();
                local_pos_pub.publish(pose);
            }
        }
            
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
