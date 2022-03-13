import os
import hillclimber

'''for i in range(0,5):
    os.system("python generate.py")
    os.system("python simulate.py")'''
hc = hillclimber.HILL_CLIMBER()
hc.Evolve()