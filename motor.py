import numpy as np
import pybullet as p
import pyrosim.pyrosim as pyrosim
import constants as c

class MOTOR:
    def __init__(self, jointName):
        self.jointName = jointName
        self.Prepare_To_Act()

    # should be Prepare_To_Act in MOTOR class according to instructions
    def Prepare_To_Act(self):
        self.amplitude = c.amp
        if self.jointName == "Torso_FrontLeg":
            self.frequency = c.freq2
        else:
            self.frequency = c.freq
        self.offset = c.offset
        self.motorValues = self.amplitude * np.sin(self.frequency * (np.linspace(0, 2*np.pi, c.maxTimeStep)) + self.offset)

    def Set_Value(self, robotId, t):
        pyrosim.Set_Motor_For_Joint(
            bodyIndex = robotId, 
            jointName = self.jointName, 
            controlMode = p.POSITION_CONTROL, 
            targetPosition = self.motorValues[t], 
            maxForce = 20)

    # save the motor values
    def Save_Values(self):
        np.save('data/' + self.jointName + '_Target_Values', self.motorValues)