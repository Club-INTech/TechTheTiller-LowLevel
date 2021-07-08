# -*- coding: utf-8 -*-

sortieScript='serialOutput/'

from sys import *
from matplotlib.pyplot import *
from PIL import *
from math import pi
from interpolation import Interpolation

Tmesures = 0.01 #en secondes

#consigneAngle = 0.6
#consignePos = 250

file = sys.argv[1]

if(len(sys.argv) >= 3):
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
    #ligne.insert(0,0)
    #ligne.insert(0,0)
    if(len(ligne)==7):
        try:
            positions.append(float(ligne[1]))
            angles.append(float(ligne[2]))
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

    if len(sys.argv) < 4:
        print("Please provide expected consigne angle")
        exit(400)

    consigne_position = float(sys.argv[3])

    time_base = [i*100 for i in range(len(positions))]
    Img = figure(figsize=(14,14))
    ax1 = subplot(211)

    ax1.plot(time_base,positions)
    ax1.plot(time_base,[consigne_position for i in range(len(positions))])
    
    Img.savefig("serialOutput/"+file+".png")
    clf()

    subplot(211)
    plot(time_base,positions)
    plot(time_base,[consigne_position for i in range(len(positions))])
    
    

if(mode == "angle" or mode == ""):
    if len(sys.argv) < 4:
        print("Please provide expected consigne angle")
        exit(400)
    
    consigne_angle = float(sys.argv[3])

    time_base = [i*100 for i in range(len(angles))]
    Img = figure(figsize=(14,14))
    ax1 = subplot(211)
    
    ax1.plot(time_base,angles)
    ax1.plot(time_base,[consigne_angle for i in range(len(angles))])
    
    Img.savefig("serialOutput/"+file+"- angles.png")
    clf()

    subplot(211)
    plot(time_base,angles)
    plot(time_base,[consigne_angle for i in range(len(angles))])
    

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
