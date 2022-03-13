import imp
import random
import numpy as np
import pyrosim.pyrosim as pyrosim
import os

class SOLUTION:
    def __init__(self):
        self.weights = (np.random.rand(3, 2)) * 2 - 1

    def Evaluate(self, option):
        self.Create_world()
        self.Create_Body()
        self.Create_Brain()
        os.system("python simulate.py " + option)
        f = open("fitness.txt", "r")
        self.fitness = float(f.read())
        f.close()

    def Mutate(self):
        randRow = random.randint(0, 2)
        randCol = random.randint(0, 1)
        self.weights[randRow, randCol] = random.random() * 2 - 1

    def Create_world(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[2, 2, 0.5] , size=[1, 1, 1])
        pyrosim.End()

    def Create_Body(self):
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

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain.nndf")

        # This particular neuron receives value from sensor stored in Torso.
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "FrontLeg")

        pyrosim.Send_Motor_Neuron(name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name = 4 , jointName = "Torso_FrontLeg")

        '''pyrosim.Send_Synapse(sourceNeuronName=0, targetNeuronName=3, weight=1.0)
        pyrosim.Send_Synapse(sourceNeuronName=1, targetNeuronName=3, weight=2.0)
        pyrosim.Send_Synapse(sourceNeuronName=0, targetNeuronName=4, weight=1.0)
        pyrosim.Send_Synapse(sourceNeuronName=2, targetNeuronName=4, weight=2.0)'''
        
        for currentRow in range(0, 3):
            for currentColumn in range(0, 2):
                pyrosim.Send_Synapse(sourceNeuronName = currentRow, targetNeuronName = currentColumn + 3, weight = self.weights[currentRow, currentColumn])

        pyrosim.End()