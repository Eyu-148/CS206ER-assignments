import pyrosim.pyrosim as pyrosim
import pybullet as p
import constants as c
from sensor import SENSOR
from motor import MOTOR

class ROBOT:
    def __init__(self):
        self.robotId = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)
        self.Prepare_To_Sense()
        self.Prepar_To_Act()

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

    def Act(self, t):
        for m in self.motors:
            self.motors[m].Set_Value(self.robotId, t)