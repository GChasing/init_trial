#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>


mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
uint8_t close_wall = 0;
sensor_msgs::LaserScan current_laserData;
uint8_t HaveARest = 0;
void laser_cb(const sensor_msgs::LaserScan::ConstPtr& msg){
    static uint16_t delay_time = 0;
    delay_time++;
    if(delay_time>2000)
        delay_time = 2000;           //程序刚开始，丢弃一些数据，以免误判断
    if(delay_time>100)
    {
        current_laserData = *msg;
        close_wall = 0;
        //ROS_INFO("laserData = %f\t%f\t%f", current_laserData.ranges[0],current_laserData.ranges[1],current_laserData.ranges[100]);
        for(uint16_t i=0;i<360;i++)
        {
            if(current_laserData.ranges[i] < 6)
            {              
                close_wall++;
            }
        }
        if(close_wall == 0)
        {
            HaveARest = 1;
        }
        if(close_wall>60)
        {
            
            close_wall = 254;
        }
    }  
}

geometry_msgs::PoseStamped current_position;
void Currentposition(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_position = *msg;
}

uint8_t FirstStepCheck(geometry_msgs::PoseStamped pose)
{
    static uint8_t e_counter = 0;
    static uint16_t sleep_counter = 0;
    sleep_counter ++;
    float e_x = current_position.pose.position.x - pose.pose.position.x;
    float e_y = current_position.pose.position.y - pose.pose.position.y;
    float e_z = current_position.pose.position.z - pose.pose.position.z; 
    if(fabs(e_x)<0.5 &&fabs(e_y)<0.5 &&fabs(e_z)<0.5 )
    {
        e_counter++;
    }
    else{
        e_counter = 0;
    }                       //连续100次
    if(e_counter>15 && sleep_counter>100)
    {
        sleep_counter = 0;
        return 1;
    }
    return 0;
}

int main(int argc, char **argv)
{
    uint8_t IsFinished;
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/2Dlidar_scan",100,laser_cb);
    ros::ServiceClient reset_gazebo_client = nh.serviceClient<std_srvs::Empty>
            ("/gazebo/reset_world");
    ros::Subscriber currentPos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose",10,Currentposition);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(100.0);  //Hz

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

    std_srvs::Empty None_mssage;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    uint16_t step_counter = 0;
    uint16_t BeginReset = 150;
    while(ros::ok()){
        BeginReset++;
        if(BeginReset>2000)
            BeginReset = 2000;
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if(BeginReset>200)
            {
                arm_cmd.request.value = true;
                if( !current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                        if( arming_client.call(arm_cmd) &&
                            arm_cmd.response.success){
                            ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }
        }
        // IsFinished = FirstStepCheck(pose);
        if(IsFinished ){
            
            pose.pose.position.y -= 1;
            step_counter ++;
            ROS_WARN("Step %d Finished!",step_counter);
            
            // reset_gazebo_client.call(None_mssage);
        }
        if(close_wall == 254 && HaveARest)
        {
            HaveARest = 0;
            pose.pose.position.y = 0;
            ROS_WARN("Begining to reset");
            arm_cmd.request.value = false;
            arming_client.call(arm_cmd);
            reset_gazebo_client.call(None_mssage);            
            BeginReset = 0;
            
        }
        //local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}