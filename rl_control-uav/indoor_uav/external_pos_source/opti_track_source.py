# -*- coding:utf-8 -*-
from external_pos_source.ext_pos_source import ext_pos_source
import socket
import math
import threading
import time
import logging

class opti_track_source(ext_pos_source):

    def __init__(self, param):
        super(opti_track_source, self).__init__(param)
        self.addr = param[0]
        self.port = param[1]

        self.position = [0, 0, 0, 0, 0, 0]
        self.lock_position = threading.Lock()

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((self.addr, self.port))

        self.update_external_pos_switch = False
        self.update_external_pos_thread = threading.Thread(target=self.update_external_position_loop)

    def start(self):
        #For isolated test
        self.update_external_pos_switch = True
        self.update_external_pos_thread.start()

    def stop(self):
        self.update_external_pos_switch = False
        time.sleep(0.5)
        self.socket.close()

    def update_position(self, pos_ext):
        '''
        @ update_position: update external position one time
        :param pos_ext: the pos_ext to be updated
        :return: updated pos_ext
        '''
        # for local test
        #'''
        pos_ext[0] = 1
        pos_ext[1] = 1
        pos_ext[2] = -0.2
        return pos_ext
        #'''
        # optitrack
        self.optitrack_data, self.optitrack_addr = self.socket.recvfrom(256)
        if not self.optitrack_data:
            logging.error('[OptiTrack] No data!')
            return pos_ext
        new_position = self.optitrack_data.split(b',')
        for idx in range(6):
            new_position[idx] = float(new_position[idx])
        '''
        logging.info('[MOCAP] {0} {1} {2} {3} {4} {5}'.format(
            new_position[0], new_position[1],
            new_position[2], new_position[3],
            new_position[4], new_position[5]))
        '''
        return new_position

    def update_external_position_loop(self):
        logging.info('Start update optitrack position thread...')
        pos_cnt = 0
        while self.update_external_pos_switch:
            self.optitrack_data, self.optitrack_addr = self.socket.recvfrom(256)
            if not self.optitrack_data:
                logging.error('no data!')
                continue
            new_position = self.optitrack_data.split(b',')
            for idx in range(6):
                new_position[idx] = float(new_position[idx])

            self.lock_position.acquire()
            self.position = new_position
            self.lock_position.release()

            pos_cnt = pos_cnt + 1
            if pos_cnt < 10:
                logging.info('[MOCAP] {0} {1} {2} {3} {4} {5}'.format(
                             self.position[0], self.position[1],
                             self.position[2], self.position[3],
                             self.position[4], self.position[5]))
        logging.info('Stop update optitrack position thread...')


if __name__ == '__main__':
    # OptiTrack network, own IP (used for opti to send pos feedback)
    opti_track = opti_track_source(['192.168.50.129', 31500])
    opti_track.start()
    while True:
        opti_track.lock_position.acquire()
        print(opti_track.position)
        opti_track.lock_position.release()
        time.sleep(0.5)
    # time.sleep(2000)