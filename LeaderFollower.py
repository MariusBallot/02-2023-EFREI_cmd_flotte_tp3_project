# -*- coding: utf-8 -*-
"""
Multi-agent leader follower simulation

(c) S. Bertrand
"""

import numpy as np
import math
import matplotlib.pyplot as plt
import Robot
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


# reference definition (with respect to elader)
d = 2
rRef = np.array([ 
  [ [ 0 ], [ 0 ] ],
  [ [ 0 ], [ d ] ],
  [ [ -d], [ 0 ] ],
  [ [ d ], [ 0 ] ],
  [ [ 0 ], [ -d ] ],
  [ [ 0], [ -d*2 ] ]
])

# gains for leader (L) and follower (F) robots
kL = 1.0  # **** A MODIFIER EN TP ****
kF = 1.0  # **** A MODIFIER EN TP ****

targetPoint = np.array([[-10], [0]])


# main loop of simulation
for t in simulation.t:

    # computation for each robot of the fleet
    for i in range(0, fleet.nbOfRobots):
        
        u = np.array([[0], [0]])
        
        if(i == 0):
            u = kL*(targetPoint - fleet.robot[i].state)
        else :
            u = kL*((fleet.robot[0].state+rRef[i]) - fleet.robot[i].state)

        # control input of robot i
        # **** A COMPLETER EN TP **** #
        fleet.robot[i].ctrl = u

    # store simulation data
    simulation.addDataFromFleet(fleet)
    # integrat motion over sampling period
    fleet.integrateMotion(Te)
    

# plots
simulation.plot(figNo=2)
simulation.plotFleet(figNo=2, mod=100, links=True)
