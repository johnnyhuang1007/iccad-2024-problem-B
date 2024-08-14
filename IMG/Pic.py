import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import matplotlib.patches as patches
import random



file_path = 'img.txt'
with open(file_path, 'r') as file:
    content = file.read()


lines = content.splitlines()
line = lines[0].split()

width = int(line[0])
height = int(line[1])

plt.figure()
plt.axis('equal')
plt.scatter(width, height,s = 0.1)


lines.pop(0)

line = lines[0].split() 
name = line[0]
num = int(line[1])
lines.pop(0)

for i in range(num):
    line = lines[0].split()
    x = int(line[0])
    y = int(line[1])
    w = int(line[2])
    h = int(line[3])
    lines.pop(0)
    rect = Rectangle((x, y), w, h,color = 'black')
    plt.gca().add_patch(rect);

line = lines[0].split()
name = line[0]
num = int(line[1])
lines.pop(0)
for i in range(num):
    line = lines[0].split()
    softname = line[0]
    tilenum = int(line[1])
    lines.pop(0)
    red = random.random()  
    green = random.random() 
    blue = random.random()  
    for j in range(tilenum):
        line = lines[0].split()
        x = int(line[0])
        y = int(line[1])
        w = int(line[2])
        h = int(line[3])
        lines.pop(0)
        rect = Rectangle((x, y), w, h)
        rect.set_facecolor((red, green, blue))
        plt.gca().add_patch(rect)

plt.gca().add_patch(rect);
plt.xlabel('Width')
plt.ylabel('Height')


plt.gca().add_patch(Rectangle((0, 0), width, height,fill = False,color = 'black',linestyle = '--'))

line = lines[0].split()
name = line[0]
num = int(line[1])
lines.pop(0)
for i in range(num):
    line = lines[0].split()
    weight = int(line[0])
    x1 = float(line[1])
    y1 = float(line[2])
    x2 = float(line[3])
    y2 = float(line[4])
    lines.pop(0)
    x_values = [x1, x2]  # x坐标的起始和结束值
    y_values = [y1, y2]  # y坐标的起始和结束值
    plt.gca().plot(x_values, y_values, color='blue', linewidth=(weight**(1/3))/10, label='Line 1')
plt.show()
plt.savefig('1.png') 