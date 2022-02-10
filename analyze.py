from cProfile import label
import numpy as np
import matplotlib.pyplot as plt

targetAnglesBack = np.load("data/targetAnglesBack.npy")
targetAnglesFront = np.load("data/targetAnglesFront.npy")
plt.plot(targetAnglesBack, label='back', linewidth=3)
plt.plot(targetAnglesFront, label='front')
plt.legend()
plt.show()
'''
frontLegSensorValues = np.load("data/backLegSensorValues.npy")
backLegSensorValues = np.load("data/frontLegSensorValues.npy")

print(frontLegSensorValues)
print(backLegSensorValues)

plt.plot(frontLegSensorValues, label='frontLeg', linewidth=1)
plt.plot(backLegSensorValues, label='backLeg')
plt.legend()
plt.show()'''