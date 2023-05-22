#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pymavlink_msgs.msg import DronePose
from datetime import datetime
import os.path
import sys
import time
import atexit

# Global variable to keep track of previous time
time_prev = time.time()

class Listener(Node):
    """
    Listener node that subscribes to drone_pose topic and logs data into a file.

    The logged data includes time, position, velocity, acceleration, and yaw of the drone.
    """

    def __init__(self):
        """
        Initialize the listener node, subscription, and log file.
        """
        super().__init__('listener')
        self.subscription = self.create_subscription(DronePose, 'drone_pose', self.callback, 10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Initiating logging')
        now = datetime.now()
        current_time = now.strftime("%m%d%y%H%M")
        path = str(os.path.expanduser('~')+'/Logs/logs/')
        if(len(sys.argv)>1):
            filename = sys.argv[1]
            filename = filename.replace(' ', '_')
            self.file = open(path + 'log' + current_time + '_' + filename + '.csv','w')
        else:
            self.file = open(path + 'log' + current_time + '.csv','w')
        self.file.write('time,x,y,z,yaw,xvel,yvel,zvel,xacc,yacc,zacc\n')

    def callback(self, data):
        """
        Callback function to process DronePose messages.

        This function logs the data into the log file.

        :param data: The received DronePose message.
        """
        time_now = time.time() - time_prev
        data_str = str(time_now) + ","+ str(data.pos.x) + ","+ str(data.pos.y) + ","+ str(data.pos.z)+ "," + str(data.yaw.data) + ',' + str(data.vel.x) + ',' + str(data.vel.y) + ',' + str(data.vel.z) + ',' + str(data.accel.x) + ',' + str(data.accel.y) + ',' + str(data.accel.z) + '\n'
        self.file.write(data_str)
        self.file.flush()

def main(args=None):
    """
    The main function to initialize and run the listener node.
    
    :param args: Command-line arguments passed to the script.
    """
    rclpy.init(args=args)
    node = Listener()
    atexit.register(exit_handler, node)
    now = datetime.now()
    current_time = now.strftime("%m%d%y%H%M")
    path = str(os.path.expanduser('~')+'/Logs/logs/')

    if(len(sys.argv)>1):
        filename = sys.argv[1]
        filename = filename.replace(' ', '_')
        file = open(path + 'log' + current_time + '_' + filename + '.csv','w')
    else:
        file = open(path + 'log' + current_time + '.csv','w')

    file.write('time,x,y,z,yaw,xvel,yvel,zvel,xacc,yacc,zacc\n')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def exit_handler(node : Listener):
    """
    Function to handle the exit of the program.

    This function closes the log file and logs that the file is closed.

    :param node: The Listener node object.
    """
    if hasattr(node, 'file'):
        node.file.close()
        node.get_logger().info('Closing log file')



if __name__ == '__main__':
    main()