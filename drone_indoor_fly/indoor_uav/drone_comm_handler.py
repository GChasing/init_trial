# -*- coding:utf-8 -*-
# communication handler for drone

class drone_comm_handler:
    def __init__(self, type):
        self.type = type

class drone_serial_handler(drone_comm_handler):
    def __init__(self, type, port, baud):
        #super(drone_comm_handler, self).__init__(type)
        drone_comm_handler.__init__(self, type)
        self.port = port
        self.baud = baud

class drone_udp_handler(drone_comm_handler):
    def __init__(self, type, url):
        drone_comm_handler.__init__(self, type)
        self.url = url