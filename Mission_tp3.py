# -*- coding: utf-8 -*-
"""
Sample script for project

(c) S. Bertrand
"""


"""
Mission objectives: 
**********************
 - reach successively the three waypoints (black stars)
 - maintain the three robots in a triangle formation as much as possible:
 
                          #0
                       /   ^  \
                      /    |d  \         (with d=6m)
                    #1 <-> v <-> #2
                       d/2   d/2
                      
 - no motion through obstacles (gray rectangles)
"""


import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import Robot
import Simulation
import math


# fleet definition
nbOfRobots = 3 
fleet = Robot.Fleet(nbOfRobots, dynamics='singleIntegrator2D')#, initState=initState)    


# initial positions
np.random.seed(100)
for i in range(0, nbOfRobots):
    fleet.robot[i].state = 10*np.random.rand(2, 1)-5  # random init btw -10, +10


# init simulation object
Te = 0.01
simulation = Simulation.FleetSimulation(fleet, t0=0.0, tf=70.0, dt=Te)


# WayPoints
WPListInit = [ np.array([[30],[0]]) , np.array([[30],[70]]),  np.array([[0],[70]])]


# obstacles
obstacle1 = patches.Rectangle((-10,20), 38, 20, color='grey', fill=True)
obstacle2 = patches.Rectangle((32,20), 8, 20, color='grey', fill=True)

print(obstacle1.get_bbox())
print(obstacle2.get_bbox())


d = 6
triRef = np.array([ 
  [ [ 0 ], [ 0 ] ],
  [ [ -d ], [ -d/2 ] ],
  [ [ -d], [ d/2 ] ],
])

lineRef = np.array([ 
  [ [ 0 ], [ 0 ] ],
  [ [ 0 ], [ -d ] ],
  [ [ 0], [ -2*d ] ],
])

kp = 1.0
kL = 1.0
kF = 1.0

distThreshold = 1
stateTracker = 0

#INITIALISATION DES VARAIBLE DE WAYPOINTS
wpIndex = 0
wpDist = 0
wpDistThreshold = .5


for t in simulation.t:


    # computation for each robot of the fleet
    if stateTracker == 0 :
              
        
        #MODE CONSENSUS AVEC CALCUL DE DISTANCE MOYENNE

        distAvg = 0
        ui = 0

        for i in range(0, fleet.nbOfRobots):
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
                stateTracker = 1
                

    if stateTracker == 1 :
        
        #MODE LEADER-FOLLOWER
        for i in range(0, fleet.nbOfRobots):
        
            u = np.array([[0], [0]])
        
            if(i == 0):
                
                if(wpIndex == 1):
                    
                    gapSize = obstacle1.get_bbox().x1 - obstacle2.get_bbox().x0
                    gapCenter = obstacle1.get_bbox().x1+gapSize/2
                    xOffset = (fleet.robot[i].state[0]-gapCenter)
                    offseter = np.array([xOffset,[0]]);
                    u = kL*(WPListInit[wpIndex] - fleet.robot[i].state)

                else:
                    
                    u = kL*(WPListInit[wpIndex] - fleet.robot[i].state)
            else :
                if(wpIndex == 1):
                    u = kF*((fleet.robot[0].state+lineRef[i]) - fleet.robot[i].state)
                else :
                    u = kF*((fleet.robot[0].state+triRef[i]) - fleet.robot[i].state)

            fleet.robot[i].ctrl = u
        
        wpDist = math.sqrt(pow(fleet.robot[0].state[0] - WPListInit[wpIndex][0], 2)+
                         pow(fleet.robot[0].state[1] - WPListInit[wpIndex][1], 2))
        if wpDist < wpDistThreshold : 
            if wpIndex < 2 :
                wpIndex+= 1
        
       
    # do not modify these two lines
    simulation.addDataFromFleet(fleet)
    fleet.integrateMotion(Te)



# plot
#simulation.plot(figNo=2)
simulation.plotFleet(figNo=2, mod=150, links=True)
fig = plt.figure(2)
fig.axes[0].set_xlim(-10, 40)
fig.axes[0].set_ylim(-10, 80)
fig.axes[0].add_patch(obstacle1)
fig.axes[0].add_patch(obstacle2)
for wp in WPListInit:
    fig.axes[0].plot(wp[0], wp[1], color='k', marker='*', markersize=15)