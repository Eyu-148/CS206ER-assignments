import numpy as np
import pyrosim.pyrosim as pyrosim
import constants as c

class SENSOR:
    def __init__(self, linkName):
        self.linkName = linkName
        # initialize sensor values vectors
        self.sensorValues = np.zeros(1000)

    # this method store the sensor value at the current time
    # step t into values[]
    def Get_Value(self, t):
        self.sensorValues[t] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)

    # save the values returned by sensors
    def Save_Values(self):
        np.save('data/' + self.linkName + '_Sensor_Values', self.sensorValues)