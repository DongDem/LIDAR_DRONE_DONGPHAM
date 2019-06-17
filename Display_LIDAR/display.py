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
        distances.append(float(line.split(",")[1])/1000)
        count = count -1
        if (count==0):
            all_angles.append(np.array(copy.deepcopy(angles)))
            all_distances.append(np.array(copy.deepcopy(distances)))
            angles.clear()
            distances.clear()
    return all_angles, all_distances


def main():
    x_drone, y_drone, id_drone = flight_path_read("./FlightPath_NEW.csv")
    x_drone = np.array(x_drone)


    all_angles, all_distances =  file_read_lidar_points("./LIDARPoints.csv")
    all_angles = np.array(all_angles)
    all_distances = np.array(all_distances)

    x_drone_new =[]
    y_drone_new =[]
    m = plt.figure()
    for i in range(all_angles.shape[0]):

        ox = np.cos(np.deg2rad(all_angles[i])) * all_distances[i]
        oy = np.sin(np.deg2rad(all_angles[i])) * all_distances[i]
        ox = ox + x_drone[i]
        oy = oy + y_drone[i]

        for j in range(ox.shape[0]):
            plt.scatter(ox[j], oy[j], s=1)
            # Visualize lidar path
           # plt.plot([ox[j],x_drone[i]], [oy[j],y_drone[i]],  "r-")

        # Show the drone position and visualize filght path
        plt.scatter(x_drone[i], y_drone[i], s=1)
        x_drone_new.append(x_drone[i])
        y_drone_new.append(y_drone[i])
        for t in range(len(x_drone_new)):

            if t>0:
                plt.scatter(x_drone_new[t-1], y_drone_new[t-1], s=1)
                plt.plot([x_drone_new[t], x_drone_new[t-1]], [y_drone_new[t], y_drone_new[t-1]], "r-")

        width = 0.3
        height = 0.2
        ax = m.add_subplot(111)
        ax.add_patch(Rectangle(xy=(x_drone[i] - width / 2, y_drone[i] - height / 2), width=width, height=height, linewidth=1, color='blue', fill=False))
        plt.axis("equal")
        plt.grid(True)
        plt.savefig('./flight_path_image/flight_path{}.png'.format(i))
        # plt.show()
        print(i)
    #plt.savefig('./Mapping.png')
   # plt.show()


if __name__ == '__main__':
    main()


