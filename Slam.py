# -*- coding: utf-8 -*-
import numpy as np
from numpy.linalg import inv
from numpy import transpose
from numpy import dot
from math import *
import matplotlib.pyplot as plt

def moveParticle(xt, m0, movementCovariance):
    v = speed = np.random.normal(m0[0], movementCovariance[0][0], 1)
    theta = rotation = np.random.normal(m0[1], movementCovariance[1][1], 1)

    delta = [cos(xt[2]+theta)*v,sin(xt[2]+theta)*v,theta]
    return xt+delta

def getMeasurement(vehiclePosition, landmarkPosition, measurementCovariance):
    deltaX = landmarkPosition.position[0] - vehiclePosition[0]
    deltaY = landmarkPosition.position[1] - vehiclePosition[1]
    deltaXY = vectorToLandmark = [deltaX, deltaY]

    normXY = np.linalg.norm(deltaXY)
    normXY += np.random.normal(0,Ez[0][0],1)

    theta = np.arctan(deltaXY[1], deltaXY[0])
    theta += np.random.normal(0,Ez[1][1],1)

    q = normXY*normXY
    deltaX = deltaX[0]
    deltaY = deltaY[0]

    H = np.array([[-deltaX/sqrt(q), -deltaY/sqrt(q), 0.0],[deltaX/q[0], deltaY/q[0], -1.0],[0.0, 0.0, -1.0]])
    z = np.array([normXY[0],theta[0],0.0])

    return z,H

def resample(oldParticles):
    weightSum = 0.0001

    for oldParticle in oldParticles:
        weightSum += oldParticle.weight

    for oldParticle in oldParticles:
        oldParticle.weight = oldParticle.weight / weightSum

    M = len(oldParticles)
    newParticles = []

    r = np.random.rand() / M
    c = oldParticles[0].weight
    i = 0

    for m in range(0,M):
        U = ( r + (m - 1) ) / M
        
        while U > c:
            i += 1
            c += oldParticles[i].weight

        newParticles.append( oldParticles[i] )

        for particle in newParticles:
            particle.weight = 1 / len(newParticles)

    return newParticles
           

timesteps = 10
maxReadDistance = 2.5
#realLandmarks = np.array([[1.0,2.0,0.0,0.0,1.0],[3.0,2.5,3.4,1.5,3.5],[0.0,0.0,0.0,0.0,0.0]])
landmarks = np.array([[1.0, 3.0, 0.0],[2.0, 2.5, 0.0],[0.0, 3.4, 0.0],[0.0, 1.5, 0.0],[1.0, 3.5, 0.0]])
landmarkList = []
realPosition = np.array([[0.0],[-1.0],[pi/3]])

movementCommand = np.array([[0.05],[0.01]]) # = m0

Ex =  np.array([[0.1,0.0],[0.0,0.05]]) # = movementCovariance
Ez = np.array([[0.1,0.0,0.0],[0.0,0.01,0.0],[0.0,0.0,0.0001]]) # = measurementCovariance

zeroVariance = np.array([[0.0,0.0],[0.0,0.0]])

numParticles = 5
particles = []

class Particle:
    def __init__(self, amount, position, landmarks):
        self.number = amount
        self.x = position[0]
        self.y = position[1]
        self.r = position[2]
        self.p = position
        self.weight = 1.0/amount
        self.landmarks = landmarks

class Landmark:
    def __init__(self, xPos, yPos):
        self.x = xPos
        self.y = yPos
        self.theta = 0
        self.position = [xPos, yPos, 0]
        self.seen = True
        self.E = np.array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]]) # Position covariance

landmarkList.append(Landmark(1.0,3.0))
landmarkList.append(Landmark(2.0,2.5))
landmarkList.append(Landmark(0.0,3.4))
landmarkList.append(Landmark(0.0,1.5))
landmarkList.append(Landmark(1.0,3.5))


for particle in range(0,numParticles):
    particles.append(Particle(numParticles, realPosition, landmarkList))


positionHistory = []

for step in range(0,timesteps):
    # Move robot to current position
    realPosition = moveParticle(realPosition, movementCommand, Ex)
    positionHistory.append(realPosition)

    # Move particle to see many possible current positions
    for particle in particles:
        particle.p = moveParticle(particle.p, movementCommand, Ex)
        particle.p[2] = realPosition[2] # Since the vehicle only got one possible heading they should stay the same

    doResample = False


    for index, landmark in enumerate(landmarkList):
        z_real, G = getMeasurement(realPosition, landmark, Ez)
        deltaL = read_distance_landmark = z_real[0]
        thetaL = read_angle_landmark = z_real[1]

        if deltaL < maxReadDistance:
            doResample = True

            for particle in particles:
                if particle.landmarks[index].seen == False:
                    particle.landmarks[index].position[0] += cos(thetaL)*deltaL
                    particle.landmarks[index].position[1] += sin(thetaL)*deltaL

                    particle.landmarks[index].E = dot( dot(inv(G),Ez) , transpose(inv(G)) )
                    particle.landmarks[index].seen == True

                else:
                    z_p, G_p = getMeasurement(particle.p, particle.landmarks[index], zeroVariance)
                    deltaZ = z_real - z_p

                    Q = dot( dot(transpose(G),particle.landmarks[index].E) , G) + Ez
                    K = dot(dot(particle.landmarks[index].E,G) , inv(Q))

                    particle.landmarks[index].position += dot(K,deltaZ)
                    particle.landmarks[index].E = dot((np.identity(3) - dot(K,transpose(G))), particle.landmarks[index].E)

                    particle.weight = particle.weight * np.power(np.linalg.norm(2*pi*Q),(-0.5*np.exp(-0.5*dot(dot(transpose(deltaZ),inv(Q)),deltaZ))))
                    
    if doResample:
        particles = resample(particles)
        print "resample"

        
    for landmark in landmarkList:
        plt.plot([landmark.x], [landmark.y], 'bs')

    for particle in particles:
        plt.plot(particle.p[0], particle.p[1], 'ro')

    print positionHistory
    xHistory = []
    yHistory = []
    
    for position in positionHistory:
        xHistory.append(position[0][0])
        yHistory.append(position[1][0])

    plt.plot(xHistory, yHistory)
        
    plt.axis([-0.5, 2.5, -2, 6])
    plt.ylabel('some numbers')
    plt.show()
