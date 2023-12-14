import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from threading import Lock

class LidarMappingNode(Node):
    def __init__(self):
        super().__init__('lidar_mapping_node')

        self.lidar1_data = None
        self.lidar2_data = None
        self.map_data = np.zeros((100, 100))  # Start mit einer 100x100-Karte
        self.map_resolution = 0.1  # Auflösung der Karte in Metern
        self.map_size = 100  
        self.robot_position = np.array([0, 0])  # Startposition des Roboters

        self.lock = Lock()

        self.create_subscription(
            LaserScan,
            '/lidar1_topic',  # Ändern Sie dies entsprechend Ihrem tatsächlichen ROS-Topic-Namen
            self.lidar1_callback,
            10)
        
        self.create_subscription(
            LaserScan,
            '/lidar2_topic',  # Ändern Sie dies entsprechend Ihrem tatsächlichen ROS-Topic-Namen
            self.lidar2_callback,
            10)

    def lidar1_callback(self, msg):
        with self.lock:
            self.lidar1_data = np.array(msg.ranges)
            self.update_map()

    def lidar2_callback(self, msg):
        with self.lock:
            self.lidar2_data = np.array(msg.ranges)
            self.update_map()

    def update_map(self):
        # Fügen Sie Ihren Code für die Kartenaktualisierung hier ein
        # Zum Beispiel: Kombinieren Sie die Daten aus self.lidar1_data und self.lidar2_data
        # Aktualisieren Sie die Karte mit den neuen Informationen

        # Überprüfen Sie, ob die Karte erweitert werden muss
        if self.lidar1_data is not None and self.lidar2_data is not None:
            max_distance = max(np.max(self.lidar1_data), np.max(self.lidar2_data))

            # Überprüfen Sie, ob die Karte erweitert werden muss
            if max_distance > self.map_size * self.map_resolution / 2:
                new_size = int(np.ceil(max_distance * 2 / self.map_resolution))
                new_map = np.zeros((new_size, new_size))

                # Berechnen Sie die Verschiebung basierend auf der Roboterposition
                center_offset = (new_size - self.map_size) // 2
                position_offset = np.array([self.robot_position[0] - center_offset,
                                           self.robot_position[1] - center_offset])

                new_map[position_offset[0]:position_offset[0] + self.map_size,
                        position_offset[1]:position_offset[1] + self.map_size] = self.map_data

                self.map_data = new_map
                self.map_size = new_size

    def display_map(self):
        # Beispiel: Zeigen Sie die Karte mit Matplotlib
        with self.lock:
            plt.imshow(self.map_data, cmap='gray')
            plt.draw()
            plt.pause(0.001)

def main(args=None):
    rclpy.init(args=args)

    lidar_mapping_node = LidarMappingNode()

    while rclpy.ok():
        rclpy.spin_once(lidar_mapping_node)
        lidar_mapping_node.display_map()

    lidar_mapping_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
