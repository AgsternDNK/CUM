import math
import numpy as np
import matplotlib.pyplot as plt
from math import cos, sin, radians, pi

#Laser Output (angles and distance data)
def file_read(f):
    measures = [line.split(",") for line in open(f)]
    angles = []
    distances = []
    for measure in measures:
        angles.append(float(measure[0]))
        distances.append(float(measure[1]))
    angles = np.array(angles)
    distances = np.array(distances)
    return angles, distances

#graph output
ang, dist = file_read("lidar01.csv")
ox = np.sin(angles) * distances
oy = np.cos(angles) * distances
plt.figure(figsize=(6,10))
plt.plot([oy, np.zeros(np.size(oy))], [ox, np.zeros(np.size(oy))], "ro-") 
plt.axis("equal")
bottom, top = plt.ylim()  # return the current ylim
plt.ylim((top, bottom)) # rescale y axis, to match the grid orientation
plt.grid(True)
plt.show()