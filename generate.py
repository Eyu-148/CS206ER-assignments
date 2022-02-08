import pyrosim.pyrosim as pyrosim

def main():
    Create_world()
    #Create_robot()
    Create_troso()


def Create_world():
    pyrosim.Start_SDF("world.sdf")
    pyrosim.Send_Cube(name="Box", pos=[2, 2, 0.5] , size=[1, 1, 1])
    pyrosim.End()

def Create_robot():
    pyrosim.Start_URDF("body.urdf")
    # pybullet use absolute coordinates from the first link and joint
    pyrosim.Send_Cube(name="Link0", pos=[0, 0, 0.5] , size=[1, 1, 1])
    pyrosim.Send_Joint(name = "Link0_Link1" , parent= "Link0" , child = "Link1" , type = "revolute", position = [0, 0, 1])

    # all next links and joints' position is relative to the last joint
    pyrosim.Send_Cube(name="Link1", pos=[0, 0, 0.5] , size=[1, 1, 1])
    pyrosim.Send_Joint(name = "Link1_Link2" , parent= "Link1" , child = "Link2" , type = "revolute", position = [0, 0, 1])
    
    pyrosim.Send_Cube(name="Link2", pos=[0, 0, 0.5] , size=[1, 1, 1])
    pyrosim.Send_Joint(name = "Link2_Link3" , parent= "Link2" , child = "Link3" , type = "revolute", position = [0, 0.5, 0.5])

    pyrosim.Send_Cube(name="Link3", pos=[0, 0.5, 0] , size=[1, 1, 1])
    pyrosim.Send_Joint(name = "Link3_Link4" , parent= "Link3" , child = "Link4" , type = "revolute", position = [0, 1, 0])

    pyrosim.Send_Cube(name="Link4", pos=[0, 0.5, 0] , size=[1, 1, 1])
    pyrosim.Send_Joint(name = "Link4_Link5" , parent= "Link4" , child = "Link5" , type = "revolute", position = [0, 0.5, -0.5])

    pyrosim.Send_Cube(name="Link5", pos=[0, 0, -0.5] , size=[1, 1, 1])
    pyrosim.Send_Joint(name = "Link5_Link6" , parent= "Link5" , child = "Link6" , type = "revolute", position = [0, 0, -1])

    pyrosim.Send_Cube(name="Link6", pos=[0, 0, -0.5] , size=[1, 1, 1])

    pyrosim.End()

# Create Troso robot in 3 (1, 1, 1) links
# Troso will the root link 
def Create_troso():
    pyrosim.Start_URDF("body.urdf")

    # Torso link - root --> absolute position
    pyrosim.Send_Cube(name="Torso", pos=[0, 0, 1.5] , size=[1, 1, 1])

    # BackLeg link --> the joint connected to the root is absolute
    pyrosim.Send_Joint(name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [-0.5, 0, 1])
    pyrosim.Send_Cube(name="BackLeg", pos=[-0.5, 0, -0.5] , size=[1, 1, 1])

    # FrontLeg link -- > the joint connected to the root is absolute
    pyrosim.Send_Joint(name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [0.5, 0, 1])
    pyrosim.Send_Cube(name="FrontLeg", pos=[0.5, 0, -0.5] , size=[1, 1, 1])

    pyrosim.End()

main()
