import os
import pyrosim.pyrosim as pyrosim
import pybullet as p
import constants as c
from sensor import SENSOR
from motor import MOTOR
from pyrosim.neuralNetwork import NEURAL_NETWORK

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
        for s in self.sensors:
            self.sensors[s].Get_Value(t)

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
        stateOfLinkZero = p.getLinkState(self.robotId, 0)
        positionOfLinkZero = stateOfLinkZero[0]
        xCoordinateOfLinkZero = positionOfLinkZero[0]
        # writing to file
        f = open("tmp" + str(self.brainID) + ".txt", "w")
        f.write(str(xCoordinateOfLinkZero))
        f.close()
        os.system("rename tmp" + str(self.brainID) + ".txt fitness" + str(self.brainID) + ".txt")