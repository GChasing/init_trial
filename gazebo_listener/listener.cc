/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include "Groundtruth.pb.h"

#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>

#include <iostream>
#include <sstream>
#include <iomanip>

#include <boost/shared_ptr.hpp>
int g_sockfd = 0;
struct sockaddr_in g_addr_pymav;



typedef const boost::shared_ptr<sensor_msgs::msgs::Groundtruth const> ConstPX4Groundtruth;
/////////////////////////////////////////////////
// Function is called everytime a message is received.
void cb(ConstWorldStatisticsPtr &_msg)
{
    // Dump the message contents to stdout.
    std::cout << _msg->DebugString();
}

void gps_cb(ConstGPSPtr &_msg)
{
    // Dump the message contents to stdout.
    std::cout << _msg->DebugString();
}

void pose_cb(ConstPosePtr &_msg)
{
    // Dump the message contents to stdout.
    std::cout << _msg->DebugString();
}

void model_cb(ConstModelPtr &_msg)
{
    // Dump the message contents to stdout.
    std::cout << _msg->DebugString();
}

void model_gt(ConstPX4Groundtruth &_msg)
{
    // Dump the message contents to stdout.
    //std::cout << _msg->DebugString();
    std::cout  << std::setiosflags(std::ios::fixed);
    std::cout  << std::setprecision(9);
    std::cout << "Latitude: " << _msg->latitude_rad() << std::endl;
    std::cout << "Longitude: " << _msg->longitude_rad() << std::endl;
    std::cout << "Altitude: " << _msg->altitude() << std::endl;

    std::stringstream sstr;
    sstr << std::setiosflags(std::ios::fixed);
    sstr << std::setprecision(9);
    sstr << _msg->latitude_rad() << ' '
         << _msg->longitude_rad() << ' '
         << _msg->altitude() << ' ';

    sendto(g_sockfd, sstr.str().c_str(), sstr.str().size(), 0, (struct sockaddr*)&g_addr_pymav, sizeof(g_addr_pymav));
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
    //Create udp socket
    g_sockfd = socket(AF_INET, SOCK_DGRAM, 0);

    //Create network connection object
    g_addr_pymav.sin_family = AF_INET;
    g_addr_pymav.sin_port = htons(23245);
    g_addr_pymav.sin_addr.s_addr = inet_addr("127.0.0.1");

    // Load gazebo
    gazebo::client::setup(_argc, _argv);

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Listen to Gazebo world_stats topic
    //gazebo::transport::SubscriberPtr sub = node->Subscribe("~/world_stats", cb);
    //gazebo::transport::SubscriberPtr sub = node->Subscribe("~/pose/info", pose_cb);
    gazebo::transport::SubscriberPtr sub = node->Subscribe("~/iris/groundtruth", model_gt);
    //gazebo::transport::SubscriberPtr sub = node->Subscribe("~/joint", gps_cb);
    //gazebo::transport::SubscriberPtr sub = node->Subscribe("~/pose/info", model_cb);
    //gazebo::transport::SubscriberPtr sub = node->Subscribe("~/iris/Odometry", gps_cb);
    //gazebo::transport::SubscriberPtr sub = node->Subscribe("~/iris/gps", gps_cb);

    // Busy wait loop...replace with your own code as needed.
    while (true)
        gazebo::common::Time::MSleep(10);

    close(g_sockfd);

    // Make sure to shut everything down.
    gazebo::client::shutdown();
}
