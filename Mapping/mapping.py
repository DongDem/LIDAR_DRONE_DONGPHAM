import math
import numpy as np
import copy
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import pandas as pd

# Read x_drone and y_drone from flight_path
def flight_path_read(f):
    x_drone = []
    y_drone = []
    id_drone = []
    for id,line in enumerate(open(f)):
        if id % 2 == 0:
            line1 = line   # line 1 for id_drone
            id_drone.append(float(line1.split(",")[0]))
        else:
            line2 = line   # line 2 for x_drone, y_drone
            x_drone.append(float(line2.split(",")[0]))
            y_drone.append(float(line2.split(",")[1]))

    return x_drone, y_drone, id_drone

# Read the angle and distance from LIDARPoints.csv
def file_read_lidar_points(f):
    all_angles = []
    all_distances = []
    angles = []
    distances = []
    for id,line in enumerate(open(f)):
        if (int(line.split(",")[1]) == 534) or (int(line.split(",")[1]) == 533):
            count = int(line.split(",")[1])
            continue
        angles.append(float(line.split(",")[0]))
        distances.append(float(line.split(",")[1]))
        count = count -1
        if (count==0):
            all_angles.append(np.array(copy.deepcopy(angles)))
            all_distances.append(np.array(copy.deepcopy(distances)))
            angles.clear()
            distances.clear()
    return all_angles, all_distances
def is_close(a,b,epsilon=5):
    return abs(a-b)<=epsilon

def main():

    x_drone, y_drone, id_drone = flight_path_read("./FlightPath_NEW.csv")
    x_drone = np.array(x_drone)
    all_angles, all_distances =  file_read_lidar_points("./LIDARPoints.csv")
    all_angles = np.array(all_angles)
    all_distances = np.array(all_distances)

    x_start = []
    y_start = []
    x_end = []
    y_end = []

    # Read lidar points for each drone position
    for i in range(all_angles.shape[0]):
        x_map = []
        y_map = []
        ox = np.cos(np.deg2rad(all_angles[i])) * all_distances[i]
        oy = np.sin(np.deg2rad(all_angles[i])) * all_distances[i]

        ox = ox + x_drone[i]*1000
        oy = oy + y_drone[i]*1000

        for j in range(ox.shape[0]):
            if j<2:
                x_map.append(int(round(ox[j])))
                y_map.append(int(round(oy[j])))
            if j>=2:
                x_map.append(int(round(ox[j])))
                y_map.append(int(round(oy[j])))
                # Remove point among start points and end points in wall
                if (is_close(y_map[-1], y_map[-2]))and (is_close(y_map[-1], y_map[-3])):
                    x_map.pop(-2)
                    y_map.pop(-2)
                if (is_close(x_map[-1], x_map[-2]))and (is_close(x_map[-1], x_map[-3])):
                    x_map.pop(-2)
                    y_map.pop(-2)
        plt.axis("equal")
        for i in range(len(x_map)-1):
            if is_close(x_map[i],x_map[i+1]):
                x_start.append(x_map[i])
                y_start.append(y_map[i])
                x_end.append(x_map[i+1])
                y_end.append(y_map[i+1])
            if is_close(y_map[i],y_map[i+1]):
                x_start.append(x_map[i])
                y_start.append(y_map[i])
                x_end.append(x_map[i+1])
                y_end.append(y_map[i+1])

    for i in range(len(x_start)):
        plt.plot([x_start[i], x_end[i]], [y_start[i], y_end[i]], "r")
    plt.grid(True)
    a = np.array((x_start,y_start,x_end,y_end)).T
    pd.DataFrame(a).to_csv("./Mapping.csv",index=False)
    plt.savefig('./Mapping.png')
    plt.show()

if __name__ == '__main__':
    main()
