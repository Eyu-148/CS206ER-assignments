import random
import numpy as np
import pyrosim.pyrosim as pyrosim
import os
import time
import constants as c
import fitness

class SOLUTION:
    def __init__(self, myID):
        self.myID = myID
        self.fitness = fitness.FITNESS(0, 0)
        #self.weights = (np.random.rand(c.numSensorNeurons, c.numMotorNeurons)) * 2 - 1
        self.weightsSensorToHidden = (np.random.rand(c.numSensorNeurons, c.numHiddenNeurons)) * 2 - 1
        self.weightsHiddenToMotor = (np.random.rand(c.numHiddenNeurons, c.numMotorNeurons)) * 2 - 1

    def Create_Environment(self):
        self.Create_world()
        self.Create_Body()

    def Start_Simulation(self, directOrGUI, sleepTime):
        self.Create_Environment()
        self.Create_Brain()
        os.system("python simulate.py " + directOrGUI + " " + str(self.myID) + " " + str(sleepTime))

    def Wait_For_Simulation_To_End(self):
        while not os.path.exists("fitness" + str(self.myID) + ".txt"):
            time.sleep(0.01)
        f = open("fitness" + str(self.myID) + ".txt", "r")
        self.fitness.floating = float(f.readline())
        self.fitness.distance = float(f.readline())
        #print(self.fitness.flightDuration, "  ", self.fitness.xCoordinate)
        f.close()
        os.system("del fitness" + str(self.myID) + ".txt")

    def Mutate(self):
        randRow = random.randint(0, c.numSensorNeurons - 1)
        randCol = random.randint(0, c.numMotorNeurons - 1)
        #self.weights[randRow, randCol] = random.random() * 2 - 1

        randRowMatrix1 = random.randint(0, c.numSensorNeurons - 1)
        randColMatrix1 = random.randint(0, c.numHiddenNeurons - 1)
        self.weightsSensorToHidden[randRowMatrix1, randColMatrix1] = random.random() * 2 - 1

        randRowMatrix2 = random.randint(0, c.numHiddenNeurons - 1)
        randColMatrix2 = random.randint(0, c.numMotorNeurons - 1)
        self.weightsHiddenToMotor[randRowMatrix2, randColMatrix2] = random.random() * 2 - 1

    def Set_ID(self, childID):
        self.myID = childID

    def Get_Fitness(self):
        return self.fitness

    def Create_world(self):
        pyrosim.Start_SDF("world.sdf")
        pyrosim.Send_Cube(name="Box", pos=[30, 30, 0.5] , size=[1, 1, 1])
        pyrosim.End()

    def Create_Body(self):
        pyrosim.Start_URDF("body.urdf")

        # Torso link - root --> absolute position
        pyrosim.Send_Cube(name="Torso", pos=[0, 0, 1] , size=[1, 1, 1])

        # FrontLeg link -- > the joint connected to the root is absolute
        pyrosim.Send_Joint(name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [0, 0.5, 1], jointAxis= "1 0 0")
        pyrosim.Send_Cube(name="FrontLeg", pos=[0, 0.5, 0] , size=[0.2, 1, 0.2])

        # FrontFeet link --> both link and joint are relative to the last joint
        pyrosim.Send_Joint(name = "FrontLeg_FrontFeet" , parent= "FrontLeg" , child = "FrontFeet" , type = "revolute", position = [0, 1, 0], jointAxis= "1 0 0")
        pyrosim.Send_Cube(name="FrontFeet", pos=[0, 0, -0.5] , size=[0.2, 0.2, 1])

        # BackLeg link --> the joint connected to the root is absolute
        pyrosim.Send_Joint(name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [0, -0.5, 1], jointAxis= "1 0 0")
        pyrosim.Send_Cube(name="BackLeg", pos=[0, -0.5, 0] , size=[0.2, 1, 0.2])

        # BackFeet link --> both joint and link are relative to upstream joint
        pyrosim.Send_Joint(name = "BackLeg_BackFeet" , parent= "BackLeg" , child = "BackFeet" , type = "revolute", position = [0, -1, 0], jointAxis= "1 0 0")
        pyrosim.Send_Cube(name="BackFeet", pos=[0, 0, -0.5] , size=[0.2, 0.2, 1])

        # LeftLeg link --> the joint connected to the root is absolute
        pyrosim.Send_Joint(name = "Torso_LeftLeg" , parent= "Torso" , child = "LeftLeg" , type = "revolute", position = [-0.5, 0, 1], jointAxis= "0 1 0")
        pyrosim.Send_Cube(name="LeftLeg", pos=[-0.5, 0, 0] , size=[1, 0.2, 0.2])

        # LeftFeet link --> both joint and link are relative to upstream joint
        pyrosim.Send_Joint(name = "LeftLeg_LeftFeet" , parent= "LeftLeg" , child = "LeftFeet" , type = "revolute", position = [-1, 0, 0], jointAxis= "0 1 0")
        pyrosim.Send_Cube(name="LeftFeet", pos=[0, 0, -0.5] , size=[0.2, 0.2, 1])

        # RightLeg link --> the joint connected to the root is absolute
        pyrosim.Send_Joint(name = "Torso_RightLeg" , parent= "Torso" , child = "RightLeg" , type = "revolute", position = [0.5, 0, 1], jointAxis= "0 1 0")
        pyrosim.Send_Cube(name="RightLeg", pos=[0.5, 0, 0] , size=[1, 0.2, 0.2])

        # RightFeet link --> both joint and link are relative to upstream joint
        pyrosim.Send_Joint(name = "RightLeg_RightFeet" , parent= "RightLeg" , child = "RightFeet" , type = "revolute", position = [1, 0, 0], jointAxis= "0 1 0")
        pyrosim.Send_Cube(name="RightFeet", pos=[0, 0, -0.5] , size=[0.2, 0.2, 1])

        pyrosim.End()

    def Create_Brain(self):
        pyrosim.Start_NeuralNetwork("brain" + str(self.myID) + ".nndf")

        # This particular neuron receives value from sensor stored in Torso.
        pyrosim.Send_Sensor_Neuron(name = 0 , linkName = "BackFeet")
        pyrosim.Send_Sensor_Neuron(name = 1 , linkName = "FrontFeet")
        pyrosim.Send_Sensor_Neuron(name = 2 , linkName = "LeftFeet")
        pyrosim.Send_Sensor_Neuron(name = 3 , linkName = "RightFeet")

        pyrosim.Send_Hidden_Neuron(name = 4)
        pyrosim.Send_Hidden_Neuron(name = 5)
        pyrosim.Send_Hidden_Neuron(name = 6)
        pyrosim.Send_Hidden_Neuron(name = 7)
        pyrosim.Send_Hidden_Neuron(name = 8)
        pyrosim.Send_Hidden_Neuron(name = 9)

        pyrosim.Send_Motor_Neuron(name = 10 , jointName = "Torso_FrontLeg")
        pyrosim.Send_Motor_Neuron(name = 11, jointName = "FrontLeg_FrontFeet")
        pyrosim.Send_Motor_Neuron(name = 12, jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name = 13, jointName = "BackLeg_BackFeet")
        pyrosim.Send_Motor_Neuron(name = 14, jointName = "Torso_LeftLeg")
        pyrosim.Send_Motor_Neuron(name = 15, jointName = "LeftLeg_LeftFeet")
        pyrosim.Send_Motor_Neuron(name = 16, jointName = "Torso_RightLeg")
        pyrosim.Send_Motor_Neuron(name = 17, jointName = "RightLeg_RightFeet")

        '''for currentRow in range(0, c.numSensorNeurons):
            for currentColumn in range(0, c.numMotorNeurons):
                pyrosim.Send_Synapse(sourceNeuronName = currentRow, 
                                    targetNeuronName = currentColumn + c.numSensorNeurons, 
                                    weight = self.weights[currentRow, currentColumn])'''

        for currentRow in range(0, c.numSensorNeurons):
            for currentColumn in range(0, c.numHiddenNeurons):
                pyrosim.Send_Synapse(sourceNeuronName = currentRow, 
                                    targetNeuronName = currentColumn + c.numSensorNeurons, 
                                    weight = self.weightsSensorToHidden[currentRow, currentColumn])
        
        for currentRow in range(0, c.numHiddenNeurons):
            for currentColumn in range(0, c.numMotorNeurons):
                pyrosim.Send_Synapse(sourceNeuronName = currentRow + c.numSensorNeurons, 
                                    targetNeuronName = currentColumn + c.numSensorNeurons + c.numHiddenNeurons, 
                                    weight = self.weightsHiddenToMotor[currentRow, currentColumn])
        pyrosim.End()