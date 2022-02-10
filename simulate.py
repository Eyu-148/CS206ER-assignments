from cmath import pi
import imp
import pybullet as p
import time
import pybullet_data
import pyrosim.pyrosim as pyrosim
import numpy as np
import random

# set up varibles
ampBack = pi/3
freqBack = 5
phaseOffsetBack = pi/4
ampFront = pi/4
freqFront = 5
phaseOffsetFront = 0

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")
pyrosim.Prepare_To_Simulate(robotId)

frontLegSensorValues = np.zeros(5000)
backLegSensorValues = np.zeros(5000)
# create motor values vector
targetAnglesBack = ampBack * np.sin(freqBack * (np.linspace(0, 2*pi, 1000)) + phaseOffsetBack)
targetAnglesFront = ampFront * np.sin(freqFront * (np.linspace(0, 2*pi, 1000)) + phaseOffsetFront)
#np.save("data/targetAnglesBack", targetAnglesBack)
#np.save("data/targetAnglesFront", targetAnglesFront)
#exit()

for i in range(0,1000):
	p.stepSimulation()
	# adding the touch sensor to 2 legs
	frontLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
	backLegSensorValues[i] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
	pyrosim.Set_Motor_For_Joint( 
		bodyIndex = robotId, 
		jointName = "Torso_FrontLeg", 
		controlMode = p.POSITION_CONTROL, 
		targetPosition = targetAnglesFront[i], 
		maxForce = 20)
	pyrosim.Set_Motor_For_Joint( 
		bodyIndex = robotId, 
		jointName = "Torso_BackLeg", 
		controlMode = p.POSITION_CONTROL, 
		targetPosition = targetAnglesBack[i], 
		maxForce = 20)
	#print(i)
	time.sleep(1/240)
p.disconnect()
print(backLegSensorValues)
np.save("data/frontLegSensorValues", frontLegSensorValues)
np.save("data/backLegSensorValues", backLegSensorValues)