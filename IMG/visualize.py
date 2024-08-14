import numpy
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import matplotlib.patches as patches
import random
import sys

def main():
    input_file = sys.argv[1]
    with open(input_file, 'r') as file:
        content = file.read()
    lines = content.splitlines()
    head = lines[0].split()

    width = int(head[0])
    height = int(head[1])
    print(f"width:{width}  height:{height}")
    fig, ax = plt.subplots()

    ax.set_xlim(0, width)
    ax.set_ylim(0, height)
    ax.set_aspect('equal', adjustable='datalim')
    plt.gca().add_patch(Rectangle((0, 0), width, height,fill = False,color = 'black',linestyle = '--'))
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
        plt.gca().add_patch(rect)

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
        print(f"{(x,y)}  {w}  {h}")
        lines.pop(0)
        rect = Rectangle((x, y), w, h,color = 'blue')
        plt.gca().add_patch(rect)


    plt.title('Visualize')
    plt.show()
if __name__ == "__main__":
    main()