#! /usr/bin/env python
import torch
import sys
sys.path.append('/home/chasing/Documents/reinmav-gym/reinmav-gym')
sys.path.append('/home/chasing/.local/lib/python2.7/site-packages/gym')
# print(sys.path)
# import gym
# import gym_reinmav
import torch.nn as nn
from itertools import count
import torch.nn.functional as F
import numpy as np
import rospy
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from geometry_msgs.msg import TwistStamped


# ENV_NAME = 'quadrotor3d-v0'
MEMORY_CAPACITY = 10000
BATCH_SIZE = 32
GAMMA = 0.9

#env = gym.make(ENV_NAME)
#env = env.unwrapped
#env.seed(1)
#s_dim = env.observation_space.shape[0]
s_dim = 10
a_dim = 1
#a_bound = env.action_space.high[0]
a_bound = 16
class ANet(nn.Module):
    def __init__(self,s_dim,a_dim):
        super(ANet,self).__init__()
        self.fc1 = nn.Linear(s_dim,64)
        self.fc1.weight.data.normal_(0,0.1)

        self.fc2 = nn.Linear(64,64)
        self.fc2.weight.data.normal_(0,0.1)

        self.out = nn.Linear(64,a_dim)
        self.out.weight.data.normal_(0,0.1)
    
    def forward(self,x):
        x = torch.tanh(self.fc1(x))
        x = torch.tanh(self.fc2(x))
        x = torch.tanh(self.out(x))

        action_value = x*a_bound
        return action_value


def choose_action(s):
    s = torch.unsqueeze(torch.FloatTensor(s), 0)
    return action_net(s)[0].detach() # ae s


# action_net = DDPG(s_dim,a_dim,a_bound)
action_net = ANet(s_dim,a_dim)


action_net.load_state_dict(torch.load('/home/chasing/Documents/reinmav-gym/test_only_altitude/trained_with_random.pt'))
action_net.eval()
# setpoint_pos = PoseStamped()
# setpoint_pos.pose.position.x = 0
# setpoint_pos.pose.position.y = 0
# setpoint_pos.pose.position.z = 2
local_position = PoseStamped()
locat_attitude_target = AttitudeTarget()
locat_attitude_target.thrust = 0.5
locat_attitude_target.type_mask = 0b10000111
arm_state = CommandBool()
vehicle_state = State()
local_vel = TwistStamped()
def local_pos_cb(msg):
    global local_position
    local_position = msg

def VehicleState_callback(msg):
    global vehicle_state 
    vehicle_state = msg

def local_vel_cb(msg):
    global local_vel
    local_vel = msg

def ros_test():
    rospy.init_node('rospy_node',anonymous=True)
    thrust_pub = rospy.Publisher('/mavros/setpoint_raw/attitude',AttitudeTarget,queue_size=1)
    state_arm_srv = rospy.ServiceProxy('/mavros/cmd/arming',CommandBool)
    state_mode_srv = rospy.ServiceProxy('/mavros/set_mode',SetMode)
    rospy.Subscriber('/mavros/state',State,VehicleState_callback)
    setpoint_pos_pub = rospy.Publisher('/mavros/setpoint_position/local',PoseStamped,queue_size=1)
    rospy.Subscriber('/mavros/local_position/pose',PoseStamped,local_pos_cb)
    rospy.Subscriber('/mavros/local_position/velocity',TwistStamped,local_vel_cb)
    rate = rospy.Rate(100)
    
    while not rospy.is_shutdown():
        # print(vehicle_state)
        # rospy.loginfo(vehicle_state.mode.)
        if vehicle_state.mode != 'OFFBOARD':
            state_mode_srv.call(custom_mode='OFFBOARD')
        else:
            # print(vehicle_state.armed)
            if vehicle_state.armed == False:
                state_arm_srv.call(True)
            else:
                p_x = local_position.pose.position.x
                p_y = local_position.pose.position.y
                p_z = local_position.pose.position.z
                q_0 = local_position.pose.orientation.w
                q_1 = local_position.pose.orientation.x
                q_2 = local_position.pose.orientation.y
                q_3 = local_position.pose.orientation.z
                v_x = local_vel.twist.linear.x
                v_y = local_vel.twist.linear.y
                v_z = local_vel.twist.linear.z
                s = torch.tensor([p_x,p_y,p_z,q_0,q_1,q_2,q_3,v_x,v_y,v_z])
                thrust_temp = choose_action(s)
                if thrust_temp < 0:
                    thrust_temp = 0
                else:
                    thrust_temp = thrust_temp/a_bound*2
                locat_attitude_target.thrust = thrust_temp
                print(thrust_temp)
                # data = rospy.wait_for_message("/mavros/local_position/pose",PoseStamped,timeout=None)
                # print(local_position.pose.position.z)
        thrust_pub.publish(locat_attitude_target)
        # setpoint_pos_pub.publish(setpoint_pos)
        rate.sleep()


# def ddpg_InOut():
#     for i in count(1):
#         s = env.reset()
#         for step in range(500):  #0.01s
#             env.render()
#             a = choose_action(s)
#             s_,r,done,_ = env.step(a.numpy())

#             # s_ = env.step(a.numpy())
#             s = s_
#             if done:
#                 break
#         print("Episode:",i,"Step",step)

# def ddpg_height_control():
    
if __name__ == "__main__":
    # ddpg_InOut()
    ros_test()
