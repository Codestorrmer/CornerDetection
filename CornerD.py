import numpy as np
import matplotlib.pyplot as plt
import math
#from sweeppy import Sweep
import itertools

#True->use LiDAR, False->use text files
MODE = False

XYSMOOTH = 3
DSMOOTH = 0
TRIMCUTOFF = 0.2
CORNERBUFFER = 20


GRAPHXY = True
GRAPHST = False
GRAPHD = False
GRAPH2D = False
CORNERST = True
CORNER2D = False

xd = []
yd = []

angle = []
distance = []

adjust = 0
adjustSet = False


startAngle = 265
endAngle = 275
adjust = 0

'''if(MODE):
        print("Using LiDAR")
        with Sweep('/dev/ttyUSB0') as sweep:
                sweep.set_motor_speed(2)
                sweep.set_sample_rate(1000)
                sweep.start_scanning()

                first = True
                for scan in itertools.islice(sweep.get_scans(),3):
                        if(not first):
                                s = scan[0]
                                for dataSample in s:
                                        ang = dataSample[0]/1000.0
                                        print("{} : {}".format(ang,dataSample[1]))

                                        if(ang>startAngle and ang<endAngle):
                                                angle.append(ang+adjust)
                                                distance.append(dataSample[1])
                                break
                        first = False

                sweep.stop_scanning()'''

        
if(not MODE):
        fx = open("angle.txt","r")
        for line in fx:
                if not adjustSet:
                        adjust= 270-float(line)
                        adjustSet=True

                angle.append(float(line)+adjust)



        fy = open("distance.txt","r")
        for line in fy:
                distance.append(float(line))


l = len(distance)

for i in range(l):
	xd.append(distance[i]*np.cos(angle[i]*np.pi/180.0))
	yd.append(distance[i]*np.sin(angle[i]*np.pi/180.0))

smooth = XYSMOOTH

xdata = []
ydata = []

#for i in range(0,smooth):
	#xdata.append(xd[i])
	#ydata.append(yd[i])


for i in range(smooth,l-smooth):
	sumX = 0
	sumY = 0
	for x in range(i-smooth,i+smooth+1):
		sumX+=xd[x]
		sumY+=yd[x]
	xdata.append(sumX/(2*smooth+1))
	ydata.append(sumY/(2*smooth+1))

lastX = xdata[len(xdata)-1]
lastY = ydata[len(ydata)-1]
xdata.append(lastX+5)
ydata.append(lastY)

#for i in range(l-smooth,l):
	#xdata.append(xd[i])
	#ydata.append(yd[i])

l = len(xdata)

if GRAPHXY:plt.plot(xdata, ydata, 'r-', label='raw')

cartD = []
d = 0;
dist = []

for i in range(0,l-1):
	cX = xdata[i+1]-xdata[i]
	cY = ydata[i+1]-ydata[i]
	s=cY/cX
	aTan = np.arctan(s)*180.0/np.pi
	if i>0:
		last = cartD[len(cartD)-1]
		curDif = abs(aTan-last)
		altDif = abs(aTan+180-last)
		if(altDif<curDif):aTan += 180
	cartD.append(aTan)
	d += np.sqrt(cX*cX+cY*cY)
	dist.append(d)


def inVal(i):
	return 0.5*(cartD[i]+cartD[i+1])*(dist[i+1]-dist[i])

smooth = DSMOOTH

length = len(dist)

sD = []

sumd = 0


for i in range(0,smooth-1):
		sumd+=inVal(i)



if(not DSMOOTH == 0):
	for i in range(0,length):
		start = 0;
		if(i>smooth):
			sumd-=inVal(i-smooth-1)
			start = i-smooth
		end = length-1
		if(i<(length-smooth)):
			sumd+=inVal(i+smooth-1)
			end = i+smooth
		startX = dist[0]
		if(start>0):startX=dist[start]
		endX = dist[length-1]
		if end<length:
			endX = dist[end]
		totD = endX-startX
		sD.append(sumd/totD)

if(DSMOOTH == 0):
	sD = cartD

if GRAPHD:
	plt.plot(dist, cartD, 'r-', label='raw')
	plt.plot(dist, sD, 'b-', label='smooth')

DSD = []
distCSD = []

for i in range(1,length):
	distCSD.append((dist[i]+dist[i-1])/2)
	cSD = float(sD[i]-sD[i-1])
	dD = float(dist[i]-dist[i-1])
	DSD.append(cSD/dD)

if GRAPH2D:plt.plot(distCSD,DSD,'r-',label='second')


mx = 0
mxPt = 0
for i in range(len(DSD)):
	if(abs(DSD[i])>mx):
		mx = abs(DSD[i])
		mxPt = distCSD[i]

cIndex = 0


for i in range(len(dist)):
	if(dist[i]>mxPt):
		cIndex = i
		break

if CORNER2D:plt.scatter(xdata[cIndex],ydata[cIndex])

def trimA(slopes, dists):
	maxVal = 0
	for s in slopes:
		if(abs(s)>maxVal):maxVal=abs(s)
	c = TRIMCUTOFF*maxVal
	while(abs(slopes[0])<c):
		slopes.pop(0)
		dists.pop(0)
	while(abs(slopes[len(slopes)-1])<c):
		slopes.pop(len(slopes)-1)
		dists.pop(len(dists)-1)
	midVal = (dists[0]+dists[len(dists)-1])/2
	return midVal

slopeTotals = []
sign = (sD[1]-sD[0])>0
xSlope = []
mx = 0

thisSlopes = [sD[1]-sD[0]]
thisDists = [dist[0],dist[1]]

for i in range(1,length-1):
	thisSlope = sD[i+1]-sD[i]
	thisSign = thisSlope>0
	if(sign==thisSign):
		thisSlopes.append(thisSlope)
		thisDists.append(dist[i+1])
	else:
		midVal = trimA(thisSlopes,thisDists)
		slopeTotals.append(sum(thisSlopes))
		aL = abs(sum(thisSlopes))
		thisSlopes = [thisSlope]
		thisDists = [dist[i],dist[i+1]]
		if(aL>mx):
			mx = aL
		sign = thisSign
		xSlope.append(midVal)

midVal = trimA(thisSlopes,thisDists)
slopeTotals.append(sum(thisSlopes))
aL = abs(sum(thisSlopes))
if(aL>mx):
	mx = aL
xSlope.append(midVal)

corners = []

for i in range(0,len(slopeTotals)):
	if(abs(slopeTotals[i]-45)<CORNERBUFFER):
		corners.append(xSlope[i])

cornerOn = 0
cornerX = []
cornerY = []

for i in range(0,len(dist)):
	if(cornerOn>=len(corners)):break
	if(dist[i]>corners[cornerOn]):
		cornerX.append((xdata[i+1]+xdata[i])/2)
		cornerY.append((ydata[i+1]+ydata[i])/2)
		cornerOn+=1


if CORNERST:plt.scatter(cornerX, cornerY)


if GRAPHST:plt.plot(xSlope, slopeTotals, 'r-', label='raw')


plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.show()
