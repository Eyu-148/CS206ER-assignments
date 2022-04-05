import os
import pyrosim.pyrosim as pyrosim
import pybullet as p
import constants as c
from sensor import SENSOR
from motor import MOTOR
from pyrosim.neuralNetwork import NEURAL_NETWORK
import numpy as np

class ROBOT:
    def __init__(self, solutionID):
        self.robotId = p.loadURDF("body.urdf")
        self.brainID = solutionID
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepar_To_Act()
        self.nn = NEURAL_NETWORK("brain" + self.brainID + ".nndf")
        os.system("del brain" + self.brainID + ".nndf")

    def Prepare_To_Sense(self):
        self.sensors = {}
        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Prepar_To_Act(self):
        self.motors = {}
        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName)

    # this method should iterate all the sensors and call get_value method
    def Sense(self, t):
        for linkName in self.sensors:
            self.sensors[linkName].Get_Value(t)

    def Think(self):
        self.nn.Update()
        #self.nn.Print()

    def Act(self):
        for neuronName in self.nn.Get_Neuron_Names():
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName)
                desireAngle = self.nn.Get_Value_of(neuronName)
                self.motors[jointName].Set_Value(self.robotId, desireAngle * c.motorJointRange)

    # this method is to get link state and record in a data file
    def Get_Fitness(self):
        # access 4 touch sensors values, compute the means
        meanOf4Sensors = np.zeros(len(self.sensors))
        
        count = 0
        for linkName in self.sensors:
            if count < len(self.sensors):
                meanOf4Sensors[count] = np.mean(self.sensors[linkName].sensorValues)
            count += 1
        # compute the mean of the 4 means
        #print(meanOf4Sensors)
        meanOfMeans = np.mean(meanOf4Sensors)

        # get the z-axis coordinate
        basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotId)
        basePosition = basePositionAndOrientation[0]
        zPosition = basePosition[2]

        # writing to file
        f = open("tmp" + str(self.brainID) + ".txt", "w")
        f.write(str(abs(meanOfMeans) + zPosition))
        f.close()
        os.system("rename tmp" + str(self.brainID) + ".txt fitness" + str(self.brainID) + ".txt")
