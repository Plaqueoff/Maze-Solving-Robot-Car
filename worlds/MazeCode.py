import os
import numpy as np
import random

file_stub = open("template.txt", "r")
#print(file_stub.read())
os.remove("empty.wbt")

targetfile = open("empty.wbt", "a")

targetfile.write(file_stub.read())

maze = np.zeros((20,20))

maze[19][19] = 1
walls = [(19,1),(18,19)]
#Prim's algorihm for perfect maze creation
while len(walls) > 0:
    index = random.randint(0,len(walls)-1)
    wall = walls[index]
    count = 0
    if (wall[0]>0):
        count += maze[wall[0]-1][wall[1]]
    if (wall[0]<19):
        count += maze[wall[0]+1][wall[1]]
    if (wall[1]>0):
        count += maze[wall[0]][wall[1]-1]
    if (wall[1]<19):
        count += maze[wall[0]][wall[1]+1]
    
    if count == 1:
        maze[wall[0]][wall[1]] = 1
        if (wall[0]>0):
            if (maze[wall[0]-1][wall[1]]==0):
                walls.append((wall[0]-1,wall[1]))
        if (wall[0]<19):
            if (maze[wall[0]+1][wall[1]]==0):
                walls.append((wall[0]+1,wall[1]))
        if (wall[1]>0):
            if (maze[wall[0]][wall[1]-1]==0):
                walls.append((wall[0],wall[1]-1))
        if (wall[1]<19):
            if (maze[wall[0]][wall[1]+1]==0):
                walls.append((wall[0],wall[1]+1))
    
    walls.remove(wall)

#Creates the ending flag
x = random.randint(0,19)
y = random.randint(0,19)

while(maze[x][y]==0):
    x = random.randint(0,19)
    y = random.randint(0,19)

object_str = ""
object_str = object_str + "YoubotFlag {\n"
object_str = object_str + "  translation " + str(x-9.5) + " 0.5 " + str(y-9.5) + "\n"
object_str = object_str + "}\n"
targetfile.write(object_str)

#Creates a box on every node where the algorithm places a wall
for row in range(20):
    for col in range(20):
        if (maze[row][col]==0 and random.random()>0.05):
            object_str = ""
            object_str = object_str + "SolidBox {\n"
            object_str = object_str + "  translation " + str(row-9.5) + " 0.5 " + str(col-9.5) + "\n"
            object_str = object_str + "  name \"box_" + str((row+1)*100+(col+66)) + "\"\n"
            object_str = object_str + "  size 1 1 1\n"
            object_str = object_str + "}\n"
            targetfile.write(object_str)

#Creates a bounding box around the whole arena
for i in range(22):
    object_str = ""
    object_str = object_str + "SolidBox {\n"
    object_str = object_str + "  translation " + str(-10.5) + " 0.5 " + str(i-10.5) + "\n"
    object_str = object_str + "  name \"box_" + str(i) + "\"\n"
    object_str = object_str + "  size 1 1 1\n"
    object_str = object_str + "}\n"
    targetfile.write(object_str)

    object_str = ""
    object_str = object_str + "SolidBox {\n"
    object_str = object_str + "  translation " + str(10.5) + " 0.5 " + str(i-10.5) + "\n"
    object_str = object_str + "  name \"box_" + str(i+22) + "\"\n"
    object_str = object_str + "  size 1 1 1\n"
    object_str = object_str + "}\n"
    targetfile.write(object_str)
for e in range(1,21):
    object_str = ""
    object_str = object_str + "SolidBox {\n"
    object_str = object_str + "  translation " + str(e-10.5) + " 0.5 " + str(-10.5) + "\n"
    object_str = object_str + "  name \"box_" + str(e+44) + "\"\n"
    object_str = object_str + "  size 1 1 1\n"
    object_str = object_str + "}\n"
    targetfile.write(object_str)

    object_str = ""
    object_str = object_str + "SolidBox {\n"
    object_str = object_str + "  translation " + str(e-10.5) + " 0.5 " + str(10.5) + "\n"
    object_str = object_str + "  name \"box_" + str(e+88) + "\"\n"
    object_str = object_str + "  size 1 1 1\n"
    object_str = object_str + "}\n"
    targetfile.write(object_str)

targetfile.close()