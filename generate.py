import pyrosim.pyrosim as pyrosim

pyrosim.Start_SDF("boxes.sdf")

# initialize the positions
x = 0
y = 0
z = 0.5
for i in range(3):
    # initializa size (1x1x1 box) and x-coordinate
    x = 0
    length = 1
    width = 1
    height = 1
    for j in range(3):
        # initialize size (1x1x1 box)
        length = 1
        width = 1
        height = 1
        for n in range(10):
            pyrosim.Send_Cube(name="Box", pos=[x, y, z] , size=[width, length, height])
            z += 1
            width = width*0.9
            length = length*0.9
            height = height*0.9
        x += 1 
    y += 1
pyrosim.End()
