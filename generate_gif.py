import imageio
import os
import matplotlib.pyplot as plt

import matplotlib.animation as animation
import re
def sorted_aphanumeric(data):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ]
    return sorted(data, key=alphanum_key)
fig = plt.figure()
images = []
for filename in sorted_aphanumeric(os.listdir("./Generate_lidar_points/image/")):
    filename = os.path.join("./Generate_lidar_points/image/", filename)
    print(filename)
    images.append(imageio.imread(filename))
imageio.mimsave('./Generate_lidar_points/image/lidar_points.gif', images,fps=2)

