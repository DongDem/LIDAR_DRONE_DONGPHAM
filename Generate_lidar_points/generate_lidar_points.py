import matplotlib.pyplot as plt
import numpy as np
import math
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

# Read dimension of wall in map
def file_read_mapping(f):
    x_start = []          # x coordinate for start points
    y_start = []          # y coordinate for start points
    x_end = []            # x coordinate for end points
    y_end = []            # y coordinate for end points
    for id,line in enumerate(open(f)):
        x_start.append(int(line.split(",")[0]))
        y_start.append(int(line.split(",")[1]))
        x_end.append(int(line.split(",")[2]))
        y_end.append(int(line.split(",")[3]))

    return x_start, y_start, x_end, y_end

# Create points array from wall
def get_point_line(x1, y1, x2, y2):
    points = []
    issteep = abs(y2-y1) > abs(x2-x1)
    if issteep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
    rev = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        rev = True
    y = y1
    for x in range(x1, x2+1 ,1):
        if issteep:
            points.append((float(y), float(x)))
        else:
            points.append((float(x), float(y)))

    # Reverse the list if the coordinates were reversed
    if rev:
        points.reverse()
    return points
# Get points in points array connect to drone
def get_lidar_points( points, reso_angle, x_drone, y_drone):
    angle, r  = [], []

    for vx, vy in zip(points[:,0], points[:,1]):
        vangle = math.atan2(vy-y_drone, vx-x_drone )          # angle from point to drone
        vr = np.hypot(vx-x_drone, vy-y_drone)                 # distance from point to drone
        angle.append(vangle)
        r.append(vr)

    # filter lidar point angle. each angle resolution only choose each point
    rx, ry, distance_save, angle_save = filter_lidar_point_angle( angle, r, reso_angle,x_drone, y_drone)

    return rx, ry, distance_save, angle_save

# Remove points after. Each angle resolution only choose each point
def filter_lidar_point_angle(angle, rangel, reso_angle,x_drone, y_drone):
    rx, ry, distance_save, angle_save = [], [], [], []

    distance = [float("inf") for _ in range(int(np.floor((np.pi * 2.0) / reso_angle)) + 1)]
    angle_true = np.zeros(len(distance))
    print(len(distance))
    for i in range(len(angle)):
        angleid = int(round(angle[i] / reso_angle))

        if distance[angleid] > rangel[i]:
            distance[angleid] = rangel[i]
            angle_true[angleid] = angle[i]
    for i in range(len(distance)):
        t = i * reso_angle
        if distance[i] != float("inf"):
            rx.append(distance[i] * np.cos(t)+x_drone)  #angle_true[i]
            ry.append(distance[i] * np.sin(t)+ y_drone)
            distance_save.append(distance[i])
            angle_save.append(t)

    return rx, ry, distance_save, angle_save

def main():
    x_start, y_start, x_end, y_end = file_read_mapping("./Generate_lidar_points/mapping.csv")
    x_drone, y_drone, id_drone = flight_path_read("./Generate_lidar_points/flight_path.csv")

    map_points = []
    reso_angle = np.deg2rad(0.6)    #each angle for one connect from drone
    plt.figure()
    for i in range(len(x_start)):
        wall = get_point_line(x_start[i],y_start[i],x_end[i],y_end[i])
        plt.plot([x_start[i], x_end[i]], [y_start[i], y_end[i]], "-g")    # show dimension of the wall in m
        map_points.extend(wall)
   # plt.savefig('./Generate_lidar_points/image/map.png')

    map_points = np.array(map_points)
    save_lidar_points = []
    for i in range(len(x_drone)):
        plt.figure()
        plt.cla()
        plt.axis("equal")
        plt.scatter(x_drone[i], y_drone[i], s=1)

        rx, ry, distance_save, angle_save = get_lidar_points(map_points, reso_angle=reso_angle, x_drone=x_drone[i],y_drone=y_drone[i])
        save_lidar_points.append([int(i), int(len(rx))])
        for (ix, iy, i_distane, i_angle) in zip(rx, ry,distance_save, angle_save):
            plt.plot([x_drone[i], ix], [y_drone[i], iy], "-r")
            save_lidar_points.append([float("{0:.5f}".format(np.rad2deg(i_angle))), float("{0:.5f}".format(i_distane))])
            #plt.scatter(ix, iy, s=1)
        plt.savefig('./Generate_lidar_points/image/lidar_{}.png'.format(i))
        #plt.show()
    a = np.array(save_lidar_points)
    pd.DataFrame(a).to_csv("./Generate_lidar_points/Lidar_points.csv",index=False)

if __name__ == '__main__':
    main()
