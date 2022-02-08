import imp
import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as np

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")
pyrosim.Prepare_To_Simulate(robotId)

frontLegSensorValues = np.zeros(5000)
backLegSensorValues = np.zeros(5000)

for i in range(0,5000):
	p.stepSimulation()
	# adding the touch sensor to 2 legs
	frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
	backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
	#print(i)
	time.sleep(1/100)
p.disconnect()
print(backLegSensorValues)
np.save("data/frontLegSensorValues", frontLegSensorValues)
np.save("data/backLegSensorValues", backLegSensorValues)