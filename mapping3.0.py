import matplotlib.pyplot as plt
import numpy as np

class QuadratischerRoboter:
    def __init__(self, initial_position=(0, 0), initial_angle=0):
        self.position = np.array(initial_position)
        self.angle = initial_angle

    def move(self, distance):
        self.position[0] += distance * np.cos(np.radians(self.angle))
        self.position[1] += distance * np.sin(np.radians(self.angle))

    def rotate(self, angle):
        self.angle += angle

    def scan_lidar(self, sensor_range=5):
        # Simuliere Lidar-Scans
        lidar1_distance = np.random.uniform(0, sensor_range)
        lidar2_distance = np.random.uniform(0, sensor_range)

        # Berechne globale Positionen der Hindernisse basierend auf Lidar-Distanzen
        obstacle1 = self.position + lidar1_distance * np.array([np.cos(np.radians(self.angle)), np.sin(np.radians(self.angle))])
        obstacle2 = self.position + lidar2_distance * np.array([np.cos(np.radians(self.angle + 90)), np.sin(np.radians(self.angle + 90))])

        return obstacle1, obstacle2

def create_map(robot, num_scans=100):
    obstacle_positions = []

    for _ in range(num_scans):
        obstacle1, obstacle2 = robot.scan_lidar()
        obstacle_positions.append(obstacle1)
        obstacle_positions.append(obstacle2)

        # Bewegung des Roboters
        robot.move(1)
        robot.rotate(10)

    return np.array(obstacle_positions)

def plot_map(obstacle_positions):
    plt.scatter(obstacle_positions[:, 0], obstacle_positions[:, 1], c='red', marker='o', label='Obstacles')
    plt.scatter(0, 0, c='green', marker='o', label='Robot Start')
    plt.legend()
    plt.title('Map Created by Lidar Scans')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    robot = QuadratischerRoboter()

    obstacle_positions = create_map(robot)
    plot_map(obstacle_positions)
