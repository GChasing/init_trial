from pymavlink import mavutil

master = mavutil.mavlink_connection('udp:0.0.0.0:{}'.format(1234))
