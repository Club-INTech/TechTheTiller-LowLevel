# -*- coding: utf-8 -*-

sortieScript='serialOutput/'

from sys import *
from matplotlib.pyplot import *
from PIL import *
from math import pi
from interpolation import Interpolation

Tmesures = 0.01 #en secondes

consigneAngle = 0.6
consignePos = 250

file = sys.argv[1]

if(len(sys.argv) == 3):
    mode = sys.argv[2]
else:
    mode = ""

fileStream = open(sortieScript+file)
fileStream.readline()
fileStream.readline()
fileStream.readline()
ligne = fileStream.readline()
positions = []
positionsY = []
vitessesY = []
abscisses = []

time=[]
angles = []
vitesseAngulaire = []

rightSpeed = []
rightSpeedSetpoint = []
leftSpeed = []
leftSpeedSetpoint = []

speeds = (leftSpeed,rightSpeed)
speedSetpoints = (leftSpeedSetpoint,rightSpeedSetpoint)

interpolation = Interpolation()

i=0

posOver = False
angleOver = False
speedOver = [False,False]

while(ligne!="DATAEND" and ligne):
    ligneEntiere = ligne
    ligne = ligne.split(",")
    ligne.insert(0,0)
    ligne.insert(0,0)
    ligne.insert(0,0)
    if(len(ligne)==7):
        try:
            positions.append((float(ligne[0])**2+float(ligne[1])**2)**0.5)
            #angles.append(float(ligne[2]))
            for i in [0,1]:
                speeds[i].append(float(ligne[3+2*i]))
                speedSetpoints[i].append(float(ligne[4+2*i]))

            if(positions[-1] > consignePos and (mode == "" or mode == "pos") and not posOver):
                print("DEPASSEMENT POS ", i*Tmesures)
                posOver = True
            elif(positions[-1] < consignePos and (mode == "" or mode == "pos") and posOver):
                posOver = False
            if(angles[-1] > consigneAngle and (mode == "" or mode == "angle") and not angleOver):
                print("DEPASSEMENT ANGLE ",i*Tmesures)
                angleOver = True
            elif(angles[-1] < consigneAngle and (mode == "" or mode == "angle") and angleOver):
                angleOver = False

            for i in [0,1]:
                if(speeds[i][-1] > speedSetpoints[i][-1] and (mode == "" or mode == "speed") and not speedOver[i]):
                    if(i==0):
                        print("DEPASSEMENT GAUCHE")
                    else:
                        print("DEPASSEMENT DROIT")
                    speedOver[i] = True
                elif(speeds[i][-1] < speedSetpoints[i][-1] and (mode == "" or mode == "speed") and speedOver[i]):
                    speedOver[i] = False


            abscisses = [i*Tmesures for i in range(len(positions))]
        except:
            print("Erreur de parsing pour "+ligneEntiere)
        i+=1

    ligne = fileStream.readline()

if(mode == "pos" or mode == ""):
    Img = figure(figsize=(14,14))
    ax1 = subplot(211)
    ax2 = subplot(212)
    ax1.plot(abscisses,positions)
    ax1.plot(abscisses,[consignePos]*len(abscisses))
    ax1.plot(abscisses,[0]*len(abscisses))
    ax2.plot(abscisses[1:-1],[(float(positions[i+1])-float(positions[i-1]))/0.002 for i in range(1,len(positions)-1)])
    Img.savefig("serialOutput/"+file+".png")
    clf()

    subplot(211)
    plot(abscisses,positions)
    plot(abscisses,[consignePos]*len(abscisses))
    plot(abscisses,[0]*len(abscisses))
    subplot(212)
    plot(abscisses[1:-1],[(float(positions[i+1])-float(positions[i-1]))/0.002 for i in range(1,len(positions)-1)])
    # plot(abscisses[1:-1],[9000*0.09]*(len(positionsY)-2))

if(mode == "angle" or mode == ""):
    Img = figure(figsize=(14,14))
    ax1 = subplot(211)
    ax2 = subplot(212)
    ax1.plot(abscisses,angles)
    ax1.plot(abscisses,[consigneAngle]*len(abscisses))
    ax2.plot(abscisses[1:-1],[(float(angles[i+1])-float(angles[i-1]))/0.002 for i in range(1,len(angles)-1)])
    Img.savefig("serialOutput/"+file+"- angles.png")
    clf()

    subplot(211)
    plot(abscisses,angles)
    plot(abscisses,[consigneAngle]*len(abscisses))
    subplot(212)
    plot(abscisses[1:-1],[(float(angles[i+1])-float(angles[i-1]))/0.002 for i in range(1,len(angles)-1)])
    plot(abscisses,[0]*len(abscisses))

if(mode == "speed" or mode == ""):


    Img = figure(figsize=(14,14))
    speedSubs = (subplot(211),subplot(212))
    
    time_base = [i*100 for i in range(len(speeds[0]))]
    # graph = []
    # new_time_base = []

    for i in [0,1]:
        
        # sample_time_base = time_base[:]
        # sample_speed = speeds[i][:]
    
        # Interpolation.set_points(x=sample_time_base, y=sample_speed)
        # Interpolation.sampling()


        # start_t, end_t = time_base[0], time_base[len(time_base) - 1]
        # point_nbr = int((end_t - start_t) / 1.0)
        # new_time_base.append([start_t + i*1.0 for i in range(point_nbr)])
    
        # graph.append([Interpolation.interpolation_output(t) for t in new_time_base[i]])

        speedSubs[i].plot(time_base, speeds[i])
        speedSubs[i].plot(time_base,speedSetpoints[i])
    
    Img.savefig("serialOutput/"+file+"- speeds.png")
    clf()

    for i in [0,1]:
        subplot(2,1,i+1)
        plot(time_base, speeds[i], 'r.')
        plot(time_base,speedSetpoints[i], 'b.')

show()
