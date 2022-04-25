from simulation import SIMULATION
import sys

directOrGUI = sys.argv[1]
solutionID = sys.argv[2]
sleepTime = sys.argv[3]

simulation = SIMULATION(directOrGUI, solutionID)
simulation.Run(sleepTime)
simulation.Get_Fitness()