import solution
import fitness
import constants as c
import copy
import os

class Parallel_HILL_CLIMBER:
    def __init__(self):
        os.system("del brain*.nndf")
        os.system("del fitness*.txt")
        self.nextAvailableID = 0
        self.parents = {}
        for i in range(0, c.populationSize):
            self.parents[i] = solution.SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1

    def Evolve(self):
        self.Evaluate(self.parents)
        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation()

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        self.Print()
        self.Select()

    def Spawn(self):
        self.children = {}
        for i in self.parents:
            self.children[i] = copy.deepcopy(self.parents[i])
            self.children[i].Set_ID(self.nextAvailableID)
            self.nextAvailableID += 1

    def Mutate(self):
        for c in self.children:
            self.children[c].Mutate()

    def Select(self):
        for key in self.parents: 
            if (self.parents[key].fitness < self.children[key].fitness):
                self.parents[key] = self.children[key]
        
    def Print(self):
        print("")
        for key in self.parents:
            print("Floating Time: [", self.parents[key].fitness.floating, self.children[key].fitness.floating, "]")
            print("Distance: [", self.parents[key].fitness.distance, self.children[key].fitness.distance, "]")

    # As we select the best one with longest flight duration & distance
    def Show_Best(self):
        bestParent = self.parents[0]
        for p in self.parents:
            if self.parents[p].fitness > bestParent.fitness:
                bestParent = self.parents[p]
        bestParent.Start_Simulation("GUI", c.sleepTime)

    def Evaluate(self, solutions):
        for i in range(0, c.populationSize):       
            solutions[i].Start_Simulation("DIRECT", 0)
        for i in range(0, c.populationSize):
            solutions[i].Wait_For_Simulation_To_End()
