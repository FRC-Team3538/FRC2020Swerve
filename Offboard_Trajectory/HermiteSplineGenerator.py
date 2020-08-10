import os
import numpy as np
import matplotlib.pyplot as plt
import math
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QSizePolicy, QLineEdit
import csv

# This isn't quite optimized, so use with caution

# (x, y, theta, phi)
# theta is where the robot is facing, phi is where the robot is headed (first derivative of spline)
poses = []
poses.append((24.6875, 8.885, 0.0, math.pi/2))
poses.append((24.6875, 33.25, 0.0, math.pi/2))
poses.append((21.75, 35, 0.0, math.pi))
poses.append((17.1458, 32.25, 0.0, math.pi))
poses.append((14, 33, 0.0, 7 * math.pi / 6))
poses.append((12.25, 32.25, 0.0, 7 * math.pi / 6))
poses.append((7, 35, 0.0, math.pi))
poses.append((2.5, 32, 0.0, 3 * math.pi / 2))
poses.append((2.5, 20, 0.0, 3 * math.pi / 2))
poses.append((6, 17.25, 0.0, 0.0))
poses.append((10, 20, 0.0, 0.0))
poses.append((13, 19.25, 0.0, math.pi / 6))
poses.append((14.875, 20, 0.0, math.pi / 6))
poses.append((20, 12.5, 0.0, 3 * math.pi / 2))
poses.append((15, 10, 0.0, math.pi))
poses.append((8, 7, 0.0, 3 * math.pi / 2))

visualPts = 100

# Robot Constants (Probably a good idea to undershoot by at least a little)
maxVel = 12 # ft / sec 
maxAcc = 12 # ft / sec / sec
wayptStpSz = 0.04 # sec

# Integral Resolution (This is a left riemann sum, so the resolution needs to be specified)
integralPts = 10000

### DON'T EDIT ANYTHING BELOW

coefficients = []
wayptLocations = []
trajectoryPts = []
saved = False

path = os.path.abspath("HermiteSplineGenerator.py")
pathParent = os.path.dirname(os.path.dirname(path))
pathRobit = os.path.join(os.path.join(pathParent, "Robot Code\src\main\cpp"), "auto")
pathTrajectories = os.path.join(pathRobit, "Trajectories")

field = plt.imread('Field.png')
fig, ax = plt.subplots()

app = QApplication([])
app.setStyle('Fusion')
app.setStyleSheet("QPushButton { margin: 1ex; }")
window = QWidget()
layout = QVBoxLayout()
button = QPushButton('Save Trajectory')
button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
text = QLineEdit()
#text.setFont(QFont("Arial",20))
layout.addWidget(button)
layout.addWidget(text)
window.setLayout(layout)
window.setMinimumSize(320, 200)

def calcCoefficients(x0, x1, dx0, dx1, ddx0, ddx1, y0, y1, dy0, dy1, ddy0, ddy1):
    ax = (-6 * x0) - (3 * dx0) - (0.5 * ddx0) + (0.5 * ddx1) - (3 * dx1) + (6 * x1)
    bx = (15 * x0) + (8 * dx0) + (1.5 * ddx0) - (ddx1) + (7 * dx1) - (15 * x1)
    cx = (-10 * x0) - (6 * dx0) - (1.5 * ddx0) + (0.5 * ddx1) - (4 * dx1) + (10 * x1)
    dx = 0.5 * ddx0
    ex = dx0
    fx = x0

    ay = (-6 * y0) - (3 * dy0) - (0.5 * ddy0) + (0.5 * ddy1) - (3 * dy1) + (6 * y1)
    by = (15 * y0) + (8 * dy0) + (1.5 * ddy0) - (ddy1) + (7 * dy1) - (15 * y1)
    cy = (-10 * y0) - (6 * dy0) - (1.5 * ddy0) + (0.5 * ddy1) - (4 * dy1) + (10 * y1)
    dy = 0.5 * ddy0
    ey = dy0
    fy = y0

    coefficients.append(((ax, bx, cx, dx, ex, fx), (ay, by, cy, dy, ey, fy)))

def calcWaypoints(totalLgt):
    if(totalLgt >= maxVel):
        v = 0.0
        t = 0.0
        d = 0.0
        df = (maxAcc/2) * ((maxVel/maxAcc)**2)
        tf = (2 * df) / maxVel
        df2 = totalLgt - df
        tf2 = (df2 - df) / maxVel
        temp = 0.0
        tempv = 0.0
        tempt = 0.0
        tempd = 0.0

        while(d < df):
            wayptLocations.append((d, t))
            tempd = d
            tempv = v
            tempt = t
            d += (v * wayptStpSz) + (0.5 * (maxAcc * (wayptStpSz**2)))
            v += maxAcc * wayptStpSz
            t += wayptStpSz

        d = tempd
        v = tempv
        t = tempt
        temp = t + wayptStpSz
        d = df
        t = tf
        d += ((temp - t) * maxVel)
        t = temp
        v = maxVel

        while(d < df2):
            wayptLocations.append((d, t))
            temp = d
            tempt = t
            d += maxVel * wayptStpSz
            t += wayptStpSz
        d = temp
        t = tempt
        temp = t + wayptStpSz
        d = df2
        t = tf2
        d += ((maxVel + ((temp - t) * -maxAcc)) / 2) * (temp - t)
        t = temp
        v = maxVel + ((temp - t) * -maxAcc)

        while(d < (totalLgt)):
            wayptLocations.append((d, t))
            temp = d
            tempv = v
            tempt = t
            d += ((v * wayptStpSz) + (0.5 * (-maxAcc * (wayptStpSz**2))))
            v -= maxAcc * wayptStpSz
            if(v < (maxVel/5.0)):
                v = (maxVel / 5.0)
            t += wayptStpSz
        d = temp
        v = tempv
        t = tempt

        if(abs(totalLgt - d) > 0.2):
            temp = t
            t += v / maxAcc
            #d += (v / 2) * (t - temp)
            d = totalLgt
            wayptLocations.append((d, t))
    else:
        v = 0.0
        t = 0.0
        d = 0.0
        df = totalLgt/2.0
        tf = (2 * df) / maxVel
        vf = tf * maxAcc

        while(d < df):
            wayptLocations.append((d, t))
            d += (v * wayptStpSz) + (0.5 * (maxAcc * (wayptStpSz**2)))
            v += maxAcc * wayptStpSz
            t += wayptStpSz
        
        temp = t + 0.25
        d += (df - d)
        t += (tf - t)
        d += (((2 * vf) + ((temp - t) * -maxAcc)) / 2) * (temp - t)
        t = temp
        v = vf + ((temp - t) * -maxAcc)

        while(d < totalLgt):
            wayptLocations.append((d, t))
            d += (v * wayptStpSz) + (0.5 * (-maxAcc * (wayptStpSz**2)))
            v += -maxAcc * wayptStpSz
            t += wayptStpSz

        if(v > 0.0):
            wayptLocations.append((d, t))
            temp = t
            t = v / maxAcc
            d = (v / 2) * (t - temp)

def calcTotArcLgt():
    totalLgt = 0.0
    for y in range(len(poses) - 1):
        for x in range(integralPts - 1):
            t = x / integralPts
            dt = 1 / integralPts
            xVal = ((5 * coefficients[y][0][0] * (t**4)) + (4 * coefficients[y][0][1] * (t**3)) + (3 * coefficients[y][0][2] * (t**2)) + (2 * coefficients[y][0][3] * (t)) + (coefficients[y][0][4]))**2
            yVal = ((5 * coefficients[y][1][0] * (t**4)) + (4 * coefficients[y][1][1] * (t**3)) + (3 * coefficients[y][1][2] * (t**2)) + (2 * coefficients[y][1][3] * (t)) + (coefficients[y][1][4]))**2
            val = math.sqrt(xVal + yVal)
            val *= dt
            totalLgt += val
    return totalLgt

def calcTrajectory():
    totalLgt = 0.0
    wayptCt = 0
    for y in range(len(poses) - 1):
        for x in range(integralPts - 1):
            t = x / integralPts
            dt = 1 / integralPts
            dxVal = ((5 * coefficients[y][0][0] * (t**4)) + (4 * coefficients[y][0][1] * (t**3)) + (3 * coefficients[y][0][2] * (t**2)) + (2 * coefficients[y][0][3] * (t)) + (coefficients[y][0][4]))**2
            dyVal = ((5 * coefficients[y][1][0] * (t**4)) + (4 * coefficients[y][1][1] * (t**3)) + (3 * coefficients[y][1][2] * (t**2)) + (2 * coefficients[y][1][3] * (t)) + (coefficients[y][1][4]))**2
            val = math.sqrt(dxVal + dyVal)
            val *= dt
            totalLgt += val
            if(wayptCt < (len(wayptLocations))):
                if(totalLgt >= wayptLocations[wayptCt][0]):
                    xVal = (coefficients[y][0][0] * (t**5)) + (coefficients[y][0][1] * (t**4)) + (coefficients[y][0][2] * (t**3)) + (coefficients[y][0][3] * (t**2)) + (coefficients[y][0][4] * t) + (coefficients[y][0][5])
                    yVal = (coefficients[y][1][0] * (t**5)) + (coefficients[y][1][1] * (t**4)) + (coefficients[y][1][2] * (t**3)) + (coefficients[y][1][3] * (t**2)) + (coefficients[y][1][4] * (t**1)) + (coefficients[y][1][5])
                    trajectoryPts.append((xVal, yVal, wayptLocations[wayptCt][1]))
                    wayptCt += 1

def generateTrajectory():
    totalLgt = calcTotArcLgt()
    calcWaypoints(totalLgt)
    calcTrajectory()
    return totalLgt

def saveTrajectory():
    global saved
    if(saved == False):
        string = text.text()
        if(len(string) == 0):
            print("Input Name")
            return
        filename = "Trajectory_" + string
        filePath = os.path.join(pathTrajectories, filename)
        with open((filePath + ".csv"), 'w', newline='') as csvfile:
            spamwriter = csv.writer(csvfile)
            spamwriter.writerow([len(poses)])
            for pose in poses:
                spamwriter.writerow([pose[0], pose[1], pose[2], pose[3]])
            for trajectory in trajectoryPts:
                spamwriter.writerow([trajectory[0], trajectory[1], trajectory[2]])
        print("SAVED\nThis is a great day for America!")
        saved = True

for x in range((len(poses) - 1)):
    scale = math.sqrt((poses[(x+1)][0] - poses[x][0])**2 + (poses[(x+1)][1] - poses[x][1])**2)
    x0 = poses[x][0]
    x1 = poses[(x+1)][0]
    dx0 = math.cos(poses[x][3]) * scale
    dx1 = math.cos(poses[(x+1)][3]) * scale
    ddx0 = 0.0
    ddx1 = 0.0
    y0 = poses[x][1]
    y1 = poses[(x+1)][1]
    dy0 = math.sin(poses[x][3]) * scale
    dy1 = math.sin(poses[(x+1)][3]) * scale
    ddy0 = 0.0
    ddy1 = 0.0
    calcCoefficients(x0, x1, dx0, dx1, ddx0, ddx1, y0, y1, dy0, dy1, ddy0, ddy1)

### GRAPHING (FOR COOL VISUALS)

xList = []
yList = []

"""
for y in range(len(poses) - 1):
    for x in range(visualPts + 1):
        t = x / visualPts
        xVal = (coefficients[y][0][0] * (t**5)) + (coefficients[y][0][1] * (t**4)) + (coefficients[y][0][2] * (t**3)) + (coefficients[y][0][3] * (t**2)) + (coefficients[y][0][4] * t) + (coefficients[y][0][5])
        yVal = (coefficients[y][1][0] * (t**5)) + (coefficients[y][1][1] * (t**4)) + (coefficients[y][1][2] * (t**3)) + (coefficients[y][1][3] * (t**2)) + (coefficients[y][1][4] * (t**1)) + (coefficients[y][1][5])
        xList.append(xVal)
        yList.append(yVal)"""

totalLength = generateTrajectory()

#print(trajectoryPts)

for point in trajectoryPts:
    xList.append(point[0])
    yList.append(point[1])

window.show()
button.clicked.connect(saveTrajectory)
button.show()

data = "Arc Length (Ft): " + str(totalLength) + " Path Time (Sec): " + str(trajectoryPts[(len(trajectoryPts) - 1)][2])

ax.plot(xList, yList, c='r', linewidth=2.25)
plt.title(data)
ax.imshow(field, extent=[0, 27, 0, 54])
plt.show()




