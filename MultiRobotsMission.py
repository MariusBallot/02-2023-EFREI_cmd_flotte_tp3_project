# -*- coding: utf-8 -*-
"""
Multi-agent consensus simulation

(c) S. Bertrand
"""


import numpy as np
import Robot
import Graph
import math
import Simulation


# fleet definition
nbOfRobots = 6
# , initState=initState)
fleet = Robot.Fleet(nbOfRobots, dynamics='singleIntegrator2D')


# random initial positions
np.random.seed(100)
for i in range(0, nbOfRobots):
    fleet.robot[i].state = 10*np.random.rand(2, 1)-5  # random init btw -5, +5


# simulation parameters
Te = 0.01
simulation = Simulation.FleetSimulation(fleet, t0=0.0, tf=20.0, dt=Te)

# control for leader follower
d = 2
rRef = np.array([ 
  [ [ 0 ], [ 0 ] ],
  [ [ 0 ], [ d ] ],
  [ [ -d], [ 0 ] ],
  [ [ d ], [ 0 ] ],
  [ [ 0 ], [ -d ] ],
  [ [ 0], [ -d*2 ] ]
])
kL = 1.0  # **** A MODIFIER EN TP ****
kF = 1.0  # **** A MODIFIER EN TP ****

# control gain for consensus
kp = 1.0  # **** A MODIFIER EN TP ****


#Initialisation du distThreshold a 1. C'est à dire quand la moyenne de la distance entre les robots est inférieur à 1,
#  nous passons au mode leader-follower par la variable distflag.
distThreshold = 1
distflag = False
targetPoint = np.array([[0], [-10]])

ui = 0
# main loop of simulation
for t in simulation.t:

    # computation for each robot of the fleet
    if distflag :
        #MODE LEADER-FOLLOWER
        for i in range(0, fleet.nbOfRobots):
        
            u = np.array([[0], [0]])
        
            if(i == 0):
                u = kL*(targetPoint - fleet.robot[i].state)
            else :
                u = kF*((fleet.robot[0].state+rRef[i]) - fleet.robot[i].state)

            fleet.robot[i].ctrl = u
        
    else :
        #MODE CONSENSUS AVEC CALCUL DE DISTANCE MOYENNE

        distAvg = 0
        for i in range(0, fleet.nbOfRobots):
            ui = 0
            dists = []
        
            for j in range(0, fleet.nbOfRobots):
                # control input of robot i
                ui += -(fleet.robot[i].state-fleet.robot[j].state)
                fleet.robot[i].ctrl = kp*ui  # **** A COMPLETER EN TP **** #
                if j != i :
                    dist = math.sqrt(pow(fleet.robot[i].state[0] - fleet.robot[j].state[0], 2)+
                                     pow(fleet.robot[i].state[1] - fleet.robot[j].state[1], 2))
                    dists.append(dist)
                    distAvg+=min(dists)
        
            distAvg /= fleet.nbOfRobots
            if distAvg < distThreshold :
                distflag = True
        
        
        
    # store simulation data
    simulation.addDataFromFleet(fleet)
    # integrat motion over sampling period
    fleet.integrateMotion(Te)
    

# plot
simulation.plot(figNo=2)
simulation.plotFleet(figNo=2)
