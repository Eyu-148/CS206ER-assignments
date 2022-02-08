import numpy as np
import matplotlib.pyplot as plt

frontLegSensorValues = np.load("data/backLegSensorValues.npy")
backLegSensorValues = np.load("data/frontLegSensorValues.npy")

print(frontLegSensorValues)
print(backLegSensorValues)

plt.plot(frontLegSensorValues, label='frontLeg', linewidth=1)
plt.plot(backLegSensorValues, label='backLeg')
plt.legend()
plt.show()