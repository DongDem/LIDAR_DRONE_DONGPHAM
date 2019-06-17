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
    x_drone, y_drone, id_drone = flight_path_read("./FlightPath.csv")
    x_drone = np.array(x_drone)
    all_angles, all_distances =  file_read_lidar_points("./LIDARPoints.csv")
    all_angles = np.array(all_angles)
    all_distances = np.array(all_distances)

    fix_x_max = 21.835118567252202           # max x coordinate in the map
    fix_x_min = 2.096554697706061
    fix_y_max = 4.243174758304495
    fix_y_min = -10.36680219132277

    x_drone_new =[]
    y_drone_new =[]

    for i in range(all_angles.shape[0]):
        m = plt.figure()
        ox = np.cos(np.deg2rad(all_angles[i])) * all_distances[i]
        oy = np.sin(np.deg2rad(all_angles[i])) * all_distances[i]
        ox = ox + x_drone[i]
        oy = oy + y_drone[i]

        if (i<8):
            delta_x = fix_x_max - np.amax(ox)
            delta_y = fix_y_max - np.amax(oy)
            ox = ox + delta_x
            oy = oy + delta_y
            x_drone[i] = float("{0:.5f}".format(x_drone[i] + delta_x))
            y_drone[i] = float("{0:.5f}".format(y_drone[i] + delta_y))
            x_drone_new.append(x_drone[i])
            y_drone_new.append(y_drone[i])
        if (i>=8) and (i<18):
            delta_x = fix_x_max - np.amax(ox)
            delta_y = fix_y_min - np.amin(oy)
            ox = ox + delta_x
            oy = oy + delta_y
            x_drone[i] = float("{0:.5f}".format(x_drone[i] + delta_x))
            y_drone[i] = float("{0:.5f}".format(y_drone[i] + delta_y))
            x_drone_new.append(x_drone[i])
            y_drone_new.append(y_drone[i])
        if (i>=18) and (i<26):
            delta_x = fix_x_min - np.amin(ox)
            delta_y = fix_y_min - np.amin(oy)
            ox = ox + delta_x
            oy = oy + delta_y
            x_drone[i] = float("{0:.5f}".format(x_drone[i] + delta_x))
            y_drone[i] = float("{0:.5f}".format(y_drone[i] + delta_y))
            x_drone_new.append(x_drone[i])
            y_drone_new.append(y_drone[i])
        if (i>=26) and (i<34):
            delta_x = fix_x_min - np.amin(ox)
            delta_y = fix_y_max - np.amax(oy)
            ox = ox + delta_x
            oy = oy + delta_y
            x_drone[i] = float("{0:.5f}".format(x_drone[i] + delta_x))
            y_drone[i] = float("{0:.5f}".format(y_drone[i] + delta_y))
            x_drone_new.append(x_drone[i])
            y_drone_new.append(y_drone[i])
        #plt.savefig('./lidar_{}.png'.format(i))
        #plt.show()

    '''
    for j in range(ox.shape[0]):
        plt.scatter(ox[j], oy[j], s=1)
        #plt.plot([ox[j],x_flight[i]], [oy[j],y_flight[i]],  "r-")
        #plt.plot([oy[j], 0], [ox[j], 0], "ro-")

    # plt.plot([oy, np.zeros(np.size(oy))], [ox, np.zeros(np.size(oy))], "ro-") # lines from 0,0 to the
    plt.axis("equal")

    plt.scatter(x_flight[i], y_flight[i], s=1)
    if len(x_flight_new)>1:
        print('dm')
        print(len(x_flight_new))
        plt.plot([x_flight_new[len(x_flight_new)-1], x_flight_new[len(x_flight_new) - 2]], [y_flight_new[len(x_flight_new)-1], y_flight_new[len(x_flight_new) - 2]], "r-")
    '''

    '''
    for t in range(len(x_flight_new)):

        if t>0:
            print('dm')
            print(x_flight_new[t])
            print(x_flight_new[t-1])
            plt.scatter(x_flight_new[t-1], y_flight_new[t-1], s=1)
            plt.plot([x_flight_new[t], x_flight_new[t-1]], [y_flight_new[t], y_flight_new[t-1]], "r-")
    '''
    '''
    width = 0.3
    height = 0.2
    ax = m.add_subplot(111)
    ax.add_patch(Rectangle(xy=(x_flight[i] - width / 2, y_flight[i] - height / 2), width=width, height=height, linewidth=1, color='blue', fill=False))

    '''
    x_drone_new = np.array(x_drone_new)
    y_drone_new = np.array(y_drone_new)
    a = np.array((x_drone_new,y_drone_new)).T
    #np.savetxt("./FlightPath_NEW.csv", a, delimiter=",")
    pd.DataFrame(a).to_csv("./FlightPath_NEW.csv", index=False)
    plt.grid(True)
   # plt.show()

if __name__ == '__main__':
    main()

