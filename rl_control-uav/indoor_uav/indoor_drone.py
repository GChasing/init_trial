# -*- coding:utf-8 -*-
from pymavlink import mavutil
import re
import time
import threading
import logging
import sys
from external_pos_source.opti_track_source import opti_track_source
import numpy as np
import math
from pyquaternion import Quaternion
from numpy import linalg

class indoor_drone:

    def __init__(self, drone_comm_handler, system_id = 1, external_pos_source = None):
        '''
        @ __init__: init object of indoor drone class
        :param drone_comm_handler: The drone communication handler, such as serial port or udp link
        :param system_id: UAV id for control
        :param external_pos_source: The external position source, such as vision position based on OptiTrack
        '''
        self.drone_comm_handler = drone_comm_handler
        self.system_id = system_id

        self.lock_position = threading.Lock()
        self.position = [0, 0, 0, 0, 0, 0]
        self.position_external = [0, 0, 0, 0, 0, 0] # pos from external position source

        # offboard control
        self.offboard_position_control_onoff = False
        self.offboard_position_rate = 40
        self.offboard_target_position = [0, 0, 0, 0, 0, 0]
        self.offboard_position_thread = threading.Thread(target=self.offboard_position_control_loop)

        # external position
        self.push_external_pos_cnt = 0
        self.external_pos_source = external_pos_source
        self.pos_push_thread = threading.Thread(target=self.push_external_position_loop)

        # read the position and attitude data in the new thread
        self.now_ = 0 
        self.read_attitude_tread_lock = 1
        self.read_attitude_thread = threading.Thread(target=self.read_attitude_data_thread_loop)

        self.first_in = 1            # set 1 for the produrce first step in
        self.t_begin = 0             # this is the begin time for the control loop
        self.w = np.zeros((3,1))
        self.thrust = 0

        #self.opti_handler = opti_track_source(['192.168.50.129', 31500])

    def read_attitude_data_thread_loop(self):
        if not self.read_attitude_tread_lock:
            while True:
                print(time.time()-self.now_)
                quat_ = self.get_current_body_attitude()
                print(quat_)
                self.now_ = time.time()

    def offboard_position_control_loop(self):
        '''
        @  offboard_position_control_loop: loop function for offboard position control thread
        '''
        logging.info('[UAV] Auto Offboard Position Control Thread Start...')
        while self.offboard_position_control_onoff:
            self.offboard_set_target_position(self.offboard_target_position[0],
                                              self.offboard_target_position[1],
                                              self.offboard_target_position[2])
            time.sleep(1 / (self.offboard_position_rate - 1) )
        logging.info('[UAV] Auto Offboard Position Control Thread Exit...')

    # def offboard_

    def connect_uav(self):
        logging.info('[UAV] Waiting for uav connect...')
        while True:
            if self.drone_comm_handler.type == 'serial':
                self.the_connection = mavutil.mavlink_connection(self.drone_comm_handler.port, self.drone_comm_handler.baud)
            elif self.drone_comm_handler.type == 'udp':
                self.the_connection = mavutil.mavlink_connection(self.drone_comm_handler.url)
            else:
                logging.error('Wrong drone communication type {0}'.format(self.drone_comm_handler.type))

            self.the_connection.wait_heartbeat()
            if self.the_connection.target_system == self.system_id:
                logging.info('Connection established, system id : {0}, component id : {1}.' \
                      .format(self.the_connection.target_system, self.the_connection.target_component))
                break

        if self.external_pos_source != None:
            self.pos_push_thread.start()

    def arm(self):
        self.the_connection.mav.command_long_send(
            self.the_connection.target_system,  # 1# autopilot system id
            self.the_connection.target_component,  # 1# autopilot component id
            400,  # command id, ARM/DISARM
            0,  # confirmation
            1,  # arm!
            0,
            0.0, 0.0, 0.0, 0.0, 0.0,  # unused parameters for this command,
            force_mavlink1=True)
        time.sleep(0.5)
        logging.info('Indoor Drone Armed!!')

    def disarm(self):
        self.the_connection.mav.command_long_send(
            self.the_connection.target_system,  # 1# autopilot system id
            self.the_connection.target_component,  # 1# autopilot component id
            400,  # command id, ARM/DISARM
            0,  # confirmation
            0,  # disarm!
            0,
            0.0, 0.0, 0.0, 0.0, 0.0,  # unused parameters for this command,
            force_mavlink1=True)
        logging.info("Indoor Drone Disarmed!!")

    def set_attitude_target(self):
        self.the_connection.mav.set_attitude_target_send(
            0,
            self.the_connection.target_system,  # 1# autopilot system id
            self.the_connection.target_component,  # 1# autopilot component id
            0b10111000,  # command id, SET_ATTITUDE_TARGET
            [0.0,  # q
            0.0,  # q
            0.0,  # q
            0.0],  # q
            self.w[0,0], -self.w[1,0], -self.w[2,0], self.thrust,  # unused parameters for this command,
            force_mavlink1=True)
        # logging.info("Set the attitude target!!") 

    def land(self):
        if self.offboard_position_control_onoff:
            self.offboard_position_control_onoff = False
            self.offboard_position_thread.join()

        timeBegin = time.time()
        cur_pos = self.get_current_local_ned_position()
        landing_position_x = cur_pos[0]
        landing_position_y = cur_pos[1]
        while True:
            timeNow = time.time()
            timeFromBegin = timeNow - timeBegin
            cur_pos = self.get_current_local_ned_position()
            self.offboard_set_target_position(landing_position_x, landing_position_y, cur_pos[2] + 0.1 * timeFromBegin)
            if(self.position[2] > -0.01):
                logging.info('[UAV] Land finished...')
                break

    def set_offboard_position_continuously(self, position):
        '''
        @ set_offboard_position_continuously: set a target position, and the position control thread will
                                               push the target position automatically
        :param position: [x, y, z, r, p, y]
        '''
        self.offboard_target_position = position

        logging.info('[UAV] === Target === x,y,z : {0}, {1}, {2}'.format(
                     self.offboard_target_position[0],
                     self.offboard_target_position[1],
                     self.offboard_target_position[2]))

        # if position control thread is not running, pull it up
        if self.offboard_position_control_onoff == False:
            self.offboard_position_control_onoff = True
            self.offboard_position_thread.start()

    def offboard_set_target_position(self, x, y, z):
        '''
        @ offboard_set_target_position: send one target position to UAV
        @ Note: you need send target position continuously to lead the UAV to the target
        :param x:
        :param y:
        :param z:
        '''
        self.the_connection.mav.set_position_target_local_ned_send(0, self.the_connection.target_system,
                                                                   self.the_connection.target_component,
                                                                   mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b110111111000,
                                                                   x, y, z, 0, 0, 0, 0, 0, 0, 0, 0, force_mavlink1=True)
    def offboard_set_target_attitude(self,target_attitude,thrust):
        self.the_connection.mav.set_attitude_target_send(0, self.the_connection.target_system,
                                                            self.the_connection.target_component,
                                                            0b00000111,
                                                            [target_attitude[0],target_attitude[1],target_attitude[2],target_attitude[3]]
                                                            ,0,0,0,thrust,
                                                            force_mavlink1=True)
    
    
    
    
    def set_mode_to_offboard(self):
        '''
        @ set_mode_to_offboard: switch UAV's mode to offboard, so that we can conduct position and velocity control
        '''
        logging.info('[UAV] Switch to Offboard Mode...')
        for i in range(100):
            self.offboard_set_target_position(0, 0, -0.5)
        
        self.the_connection.mav.command_long_send(
            self.the_connection.target_system,  # 1# autopilot system id
            self.the_connection.target_component,  # 1# autopilot component id
            176,  # MAV_CMD_DO_SET_MODE (176 )
            0,  # confirmation
            129,
            6,
            0,
            0.0, 0.0, 0.0, 0.0,  # unused parameters for this command,
            force_mavlink1=True)
        self.read_attitude_tread_lock = 1
        self.read_attitude_thread.start()
        while True:
            self.uav_geometric_control_circle()

    def offboard_to_certain_height(self, height):
        '''
        @ offboard_to_certain_height: Lead the UAV to a certain height (with current x, y)
        :param height: target height in m
        '''
        logging.info('[UAV] Hovering at certain height...')
        cur_ned_pos = self.get_current_local_ned_position()
        cur_ned_pos[0] = 0
        cur_ned_pos[1] = 0
        cur_ned_pos[2] = -1 * height
        self.set_offboard_position_continuously(cur_ned_pos)

    def get_current_body_attitude(self):
        '''
        @get the current body attitude 
        :return the body attitude in quaternion form            !!here I add the minus to y and z axis, transform from mavlink to ros
        '''                                  
        self.the_connection.recv_match(type='ATTITUDE_QUATERNION',blocking=True)
        return np.array([[self.the_connection.messages['ATTITUDE_QUATERNION'].q1,
                    self.the_connection.messages['ATTITUDE_QUATERNION'].q2,
                    -self.the_connection.messages['ATTITUDE_QUATERNION'].q3,                    
                    -self.the_connection.messages['ATTITUDE_QUATERNION'].q4]])

    def get_current_local_ned_position(self):
        '''
        @ get_current_local_ned_position: get current LOCAL_POSITION_NED
        :return: LOCAL_POSITION_NED [x, y, z, Vx, Vy, Vz]
        '''
        # Update LOCAL_POSITION_NED
        self.the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        #'''
        # logging.info('[TELEMETRY] Local NED pos {0} {1} {2} {3} {4} {5}'.format(self.the_connection.messages['LOCAL_POSITION_NED'].x,
        #        self.the_connection.messages['LOCAL_POSITION_NED'].y,
        #        self.the_connection.messages['LOCAL_POSITION_NED'].z,
        #        self.the_connection.messages['LOCAL_POSITION_NED'].vx,
        #        self.the_connection.messages['LOCAL_POSITION_NED'].vy,
        #        self.the_connection.messages['LOCAL_POSITION_NED'].vz))            !!here x=y ,y=x z = -z
        #'''
        return np.array([[self.the_connection.messages['LOCAL_POSITION_NED'].x,
                -self.the_connection.messages['LOCAL_POSITION_NED'].y,
                -self.the_connection.messages['LOCAL_POSITION_NED'].z]]),np.array([[self.the_connection.messages['LOCAL_POSITION_NED'].vx,
                -self.the_connection.messages['LOCAL_POSITION_NED'].vy,
                -self.the_connection.messages['LOCAL_POSITION_NED'].vz]])


    def attcontroller(self,quat_des,quat_current):
        quat_curr = Quaternion(quat_current[0])
        error_att = quat_curr.inverse * quat_des
        qe = error_att
        tau = 0.1
        w = (2/tau) * np.sign(qe[0])*np.array([[qe[1],qe[2],qe[3]]]).T
        return w

    def uav_geometric_control_circle(self):
        T = 10
        w_t = 2 * math.pi / T
        r_ = 3
        if self.first_in:
            self.first_in = 0
            self.t_begin = time.time()
        else:
            t_s = time.time() - self.t_begin
            pos_target = np.array([[r_*math.sin(w_t*t_s),      r_*math.cos(w_t*t_s) ,    2]])
            vel_target = np.array([[r_*w_t*math.cos(w_t*t_s),      -r_*w_t*math.sin(w_t*t_s),    0]])
            acc_target = np.array([[-r_*w_t*w_t*math.sin(w_t*t_s),     -r_*w_t*w_t*math.cos(w_t*t_s),    0]])

            # pos_target = np.array([[10,      0 ,    2]])
            # vel_target = np.array([[0*math.cos(t_s),      -0*math.sin(t_s),    0]])
            # acc_target = np.array([[-0*math.sin(t_s),     -0*math.cos(t_s),    0]])

            pos_curr,vel_curr =  self.get_current_local_ned_position()
            # print(vel_curr)

            pos_e = pos_curr - pos_target
            vel_e = vel_curr - vel_target
            
            k_pos = np.array([[-8, -8, -10]])
            k_vel = np.array([[-1.5, -3.3,-3.3]])

            quat_current = self.get_current_body_attitude()    # in the body frame: front , right , down

            att_quat_current = Quaternion(quat_current[0])
            rotation_current = att_quat_current.rotation_matrix

            psi = 0

            a_fb = np.dot(np.diag(k_pos[0]),pos_e.T) + np.dot(np.diag(k_vel[0]),vel_e.T)
            if linalg.norm(a_fb,2) >8 :
                a_fb = (8 / linalg.norm(a_fb))*a_fb

            a_des = a_fb + 9.8*np.array([[0,0,1]]).T + acc_target.T
            Re3 = np.dot(rotation_current,np.array([[0,0,1]]).T)
            self.thrust = min(1,max(0.05*np.sum(np.multiply(a_des,Re3)) + 0.1,0)) 

            proj_xb_des = np.array([[math.cos(psi),math.sin(psi),0]])

            zb_des = (a_des / linalg.norm(a_des)).T
            yb_des = np.cross(zb_des,proj_xb_des) / linalg.norm(np.cross(zb_des,proj_xb_des))
            xb_des = np.cross(yb_des,zb_des) /linalg.norm(np.cross(yb_des,zb_des))
            R_des = np.hstack((xb_des.T,yb_des.T,zb_des.T))

            quat_des = Quaternion._from_matrix(R_des)
            yaw,pitch,roll = Quaternion.yaw_pitch_roll(quat_des)
            print(roll*180/3.14,pitch*180/3.14,yaw*180/3.14,self.thrust)

            self.w = self.attcontroller(quat_des,quat_current)
            self.set_attitude_target()


    def request_data_stream(self):
        logging.info('[UAV] Request data streams...')
        '''
        self.the_connection.mav.command_long_send(
            self.the_connection.target_system,  # 1# autopilot system id
            self.the_connection.target_component,  # 1# autopilot component id
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # command id
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            10,
            0.0, 0.0, 0.0, 0.0, 0.0,  # unused parameters for this command,
            force_mavlink1=True)
        
        self.the_connection.mav.request_data_stream_send(
            self.the_connection.target_system,  # 1# autopilot system id
            self.the_connection.target_component,  # 1# autopilot component id
            mavutil.mavlink.MAV_DATA_STREAM_ALL,  # command id
            1,
            1
        )
        '''
        time.sleep(1)

        # self.the_connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        while True:
            time.sleep(0.1)
            msg = self.the_connection.recv_match(blocking=True)
            # logging.info('[UAV] Msg type {0}'.format(msg.get_type()))
            if msg.get_type() == 'LOCAL_POSITION_NED':
                logging.info('[UAV] Ned {0} {1} {2}'.format(msg.x, msg.y, msg.z))

                logging.info('[UAV] Local NED pos {0} {1} {2}'.format(self.the_connection.messages['LOCAL_POSITION_NED'].x,
                                                                   self.the_connection.messages['LOCAL_POSITION_NED'].y,
                                                                   self.the_connection.messages[
                                                                   'LOCAL_POSITION_NED'].z))
                break
        logging.info('[UAV] After request data streams...')

    def push_external_position_loop(self):
        logging.info('[UAV] Start pushing external position to UAV...')
        while True:
            self.update_external_postion()
            self.mav_send_external_position()
            time.sleep(0.0005)

    def update_external_postion(self):
        '''
        @ update_external_postion: update external position
        '''
        # cur_ned_pos = self.get_current_local_ned_position()
        self.lock_position.acquire()
        # external_pos_source should realize the update_position method
        self.position_external = self.external_pos_source.update_position(self.position_external)
        self.lock_position.release()

    def mav_send_external_position(self):
        self.the_connection.mav.vision_position_estimate_send(self.system_id, self.position_external[0], self.position_external[1],
                                                              self.position_external[2], self.position_external[3], self.position_external[4],
                                                              self.position_external[5])
        self.push_external_pos_cnt = self.push_external_pos_cnt + 1
        '''
        if self.push_external_pos_cnt % 100 == 0:
            logging.info('[UAV] Send Mav Pos {0} {1} {2} {3} {4} {5}'.format(
                self.position_external[0], self.position_external[1],
                self.position_external[2], self.position_external[3],
                self.position_external[4], self.position_external[5]))
        '''