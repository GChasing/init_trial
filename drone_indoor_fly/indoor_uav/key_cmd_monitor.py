import threading
import logging
import time

#  key_monitor = key_cmd_monitor.key_cmd_monitor(main_exit_event, drone_fly, net_source_handler)

class key_cmd_monitor:
    def __init__(self, exit_event, uav, ext_source = None):
        self.exit_event = exit_event
        self.uav = uav
        self.monitor_thread = threading.Thread(target = self.key_cmd_monitor_loop)
        self.ext_source = ext_source

    def start(self):
        self.monitor_thread.start()

    def key_cmd_monitor_loop(self):
        while True:
            keyboard_cmd = input()
            if keyboard_cmd == 'uav':
                logging.info('[CMD] Connect uav...')
                self.uav.connect_uav()
            if keyboard_cmd == 'arm':
                logging.info('[CMD] Arm uav...')
                self.uav.arm()
            if keyboard_cmd == 'dis':
                logging.info('[CMD] Disarm uav...')
                self.uav.disarm()
            if keyboard_cmd == 'offboard':
                '''
                Note: We must push local target position to pixhawk before switch to Offboard model.
                      Use 'pos' command to set continuously target position. 
                '''
                logging.info('[CMD] Switch uav to offboard...')
                self.uav.set_mode_to_offboard()
            if keyboard_cmd == 'land':
                logging.info('[CMD] Land uav...')
                self.uav.land()
            if keyboard_cmd == 'height':
                logging.info('[CMD] Please input target height...')
                height = input()
                self.uav.offboard_to_certain_height(float(height))
            if keyboard_cmd == 'ned':
                logging.info('[CMD] Acquire uav local ned pos...')
                self.uav.get_current_local_ned_position()

            if keyboard_cmd == 'control':
                logging.info('[CMD] Controller begin')
                while True:
                    self.uav.uav_geometric_control_circle()
            # if keyboard_cmd == 'quat':
            #     logging.info('[CMD] Acquire uav local quat...')

            if keyboard_cmd == 'require':
                logging.info('[CMD] Offboard position control...')
                self.uav.request_data_stream()
            if keyboard_cmd == 'pos':
                logging.info('[CMD] Please input target position [x y z]')
                target_pos_str = input()
                target_pos = list(map(float, target_pos_str.split()))
                logging.info('[CMD] Offboard position control target %f %f %f'.format(target_pos[0],
                             target_pos[1], target_pos[2]))
                if len(target_pos) != 3:
                    logging.info('Wrong target length.')
                    continue
                if target_pos[2] < 0:
                    logging.info('Wrong target height.')
                    continue
                target_pos[2] = target_pos[2] * -1
                target_pos = target_pos + [0, 0, 0]
                self.uav.set_offboard_position_continuously(target_pos)
            if keyboard_cmd == 'opti-start':
                logging.info('[CMD] External source start...')
                self.ext_source.start()
            if keyboard_cmd == 'opti-stop':
                logging.info('[CMD] External source start...')
                self.ext_source.stop()
            if keyboard_cmd == 'square':
                logging.info('[CMD] Move square...')
                cnt = 0
                while cnt < 5:
                    target_pos = [1, 0, -1, 0, 0, 0]
                    self.uav.set_offboard_position_continuously(target_pos)
                    time.sleep(7)
                    target_pos = [1, 1, -1, 0, 0, 0]
                    self.uav.set_offboard_position_continuously(target_pos)
                    time.sleep(7)
                    target_pos = [0, 1, -1, 0, 0, 0]
                    self.uav.set_offboard_position_continuously(target_pos)
                    time.sleep(7)
                    target_pos = [0, 0, -1, 0, 0, 0]
                    self.uav.set_offboard_position_continuously(target_pos)
                    time.sleep(7)
                    cnt += 1
                self.ext_source.stop()
            if keyboard_cmd == 'exit':
                self.exit_event.set()
                break

