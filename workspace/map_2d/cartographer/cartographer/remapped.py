#!/usr/bin/python3
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
# ***************
#   OBSTACLE
M = 75
#   unknown
N = 50
#   free
# ----0-----
#   unknown
# ***************

class Remapped(Node):

    def __init__(self):
        super().__init__('remapped')
        self.cmap_sub = self.create_subscription(
            OccupancyGrid, '/cmap', self.callback, qos_profile_sensor_data
            )
            
        self.map_pub = self.create_publisher(
            OccupancyGrid, '/map', 10
            )
 
    def callback(self, msg):
        # print("got map")
        data = list(msg.data)
        cmap = OccupancyGrid()
        for y in range(msg.info.height):
            for x in range(msg.info.width):
                i = x + (msg.info.height - 1 - y) * msg.info.width
                if data[i] >= M:  
                    data[i] = 100
                elif (data[i] >= 0) and (data[i] < N):  # free
                    data[i] = 0
                else:  # unknown
                    data[i] = -1
        cmap.info = msg.info
        cmap.data = tuple(data)
        cmap.header.stamp = msg.header.stamp
        cmap.header.frame_id = "map"
        self.map_pub.publish(cmap)

def main(args=None):
    rclpy.init(args=args)

    remapped = Remapped()
    rclpy.spin(remapped)


if __name__ == '__main__':
    main()