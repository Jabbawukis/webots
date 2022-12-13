"""Beispiel fuer die Interaktion mit der Box."""

from controller import Supervisor

# https://numpy.org/doc/stable/reference/
import numpy as np  # matrizen-bibliothek, aehnlich zu Matlab.
import math as m
import time

# der Roboter hat jetzt Zugriff auf die Umwelt.
robot = Supervisor()

# Der kleinste Zeitschritt der Simulierten Welt.
timestep = int(robot.getBasicTimeStep())
output = 0.0

# laenge der joints von base aus
armBaseCords = [0.19, 0, 0.147]
armjointslen = [0.155, 0.135, 0.2175]

# referenzen zu der globalen Position des Roboters und der Box.
box = robot.getFromDef('PRODUCT')
target = robot.getFromDef('TARGET')
gps = robot.getFromDef('ROBOT.ORIGIN')

# Motoren der Raeder.
wheels = [
    robot.getDevice("wheel1"),  # vorn rechts
    robot.getDevice("wheel2"),  # vorn links
    robot.getDevice("wheel3"),  # hinten rechts
    robot.getDevice("wheel4")]  # hinten links

# motoren der Finger
fingenrs = [
    robot.getDevice("finger1"),
    robot.getDevice("finger2")
]

# motoren der Gelaenke
joints = [
    robot.getDevice("arm1"),
    robot.getDevice("arm2"),
    robot.getDevice("arm3"),
    robot.getDevice("arm4"),
    robot.getDevice("arm5")
]

# Winkelsensoren der Gelenke
jointSensors = [
    robot.getDevice("arm1sensor"),
    robot.getDevice("arm2sensor"),
    robot.getDevice("arm3sensor"),
    robot.getDevice("arm4sensor"),
    robot.getDevice("arm5sensor")
]

# Geschwindigkeit Arm-Motoren
for j in joints:
    j.setVelocity(0.5)

# Geschwindigkeit Finger-Motoren
for j in fingenrs:
    j.setVelocity(0.5)

# aktiviere sensoren
for s in jointSensors:
    s.enable(timestep)

# Positionssensoren der Fingermotoren
fingerSensors = [
    robot.getDevice("finger1sensor"),
    robot.getDevice("finger2sensor")
]

# aktiviere sensoren
for s in fingerSensors:
    s.enable(timestep)


# Hilfsfunktionen fuer die finger
def openFingers():
    position = fingenrs[0].getMaxPosition()
    fingenrs[0].setPosition(position)
    fingenrs[1].setPosition(position)


def closeFingers():
    fingenrs[0].setPosition(0)
    fingenrs[1].setPosition(0)


# Bewegungsmatrix (zurueck, nach rechts, rechts drehend)

# Wir wollen die Geschwindigkeich der Raeder steuern.
# Setze die Zielposition der Rad-Motoren auf unendlich,
# damit sie sich immer drehen wenn Geschwindigkeit nicht 0 ist.
# Setze die initiale Geschwindigkeit auf 0.
for wheel in wheels:
    wheel.setPosition(float('+inf'))
    wheel.setVelocity(0.0)
    pass

# Bewegungsmatrix des Robots, Spaltenvektoren:
# 1. vor / zurueck
# 2. links / rechts
# 3. links/ rechts Rotation um z-Achse
w = 1
lr = 1
r = 1

M = np.array(
    [[w, lr, r],
     [w, -lr, -r],
     [w, -lr, r],
     [w, lr, -r]]
)


def setSpeed(v):
    maxAbs = abs(max(np.max(v), np.min(v), key=abs))
    # Nutze maximale Geschwindigkeit und behalte Proportionale Anteile bei,
    # solange Abstand größer 1 Meter
    for i in range(len(wheels)):
        if (maxAbs > 1):
            wheels[i].setVelocity(float(14.8 * (v[i] / maxAbs)))
            # Bremsen zu präzisen Positionierung
        else:
            wheels[i].setVelocity(float(14.8 * v[i]))


def stop():
    for wheel in wheels:
        wheel.setVelocity(0.0)


def calcVecGreenBox(t, tb, R):
    # Transformation des Richtungsvektors in lokales Koord.-system des Robots
    vR = np.matmul(R.transpose(), (tb - t))
    # Umrechnung Richtungsvektor in Polarkoordinaten
    rot = np.arctan2(vR[1], vR[0])
    dist = np.linalg.norm(vR) - 0.45
    # Vektor zu Geschwindigkeitssteuerung
    p = np.array(
        [np.cos(rot) * dist,
         np.sin(rot) * dist,
         rot])
    return p


def calcVecRedBox(t, tb, R):
    # Transformation des Richtungsvektors in lokales Koord.-system des Robots
    vR = np.matmul(R.transpose(), (tb - t))
    # Umrechnung Richtungsvektor in Polarkoordinaten
    rot = np.arctan2(vR[1], vR[0])
    dist = np.linalg.norm(vR) - 0.6
    # Vektor zu Geschwindigkeitssteuerung
    p = np.array(
        [np.cos(rot) * dist,
         np.sin(rot) * dist,
         rot])
    return p


def moveArm(p):
    # Bewege Arm
    for i in range(0, 5):
        joints[i].setPosition(p[i])


def reachedArm(p):
    error = 0
    # Prüfe ob Arm in Position
    for i in range(0, 5):
        e = np.abs(jointSensors[i].getValue() - p[i])
        error = np.max([error, e])

    # maximaler Fehler unter 1 Grad = Winkel erreicht
    return error < np.radians(1.0)


def check_if_drop(t, R, tb, Rb):
    gripperpos = getGripperPos(t, R, tb, Rb)

    lokalboxpos = getLokalBoxPos(t, R, tb)
    # prüfe ob Box noch im Greifer
    if (np.linalg.norm(gripperpos - lokalboxpos) > 0.05):
        return True
    else:
        return False


def getLokalBoxPos(t, R, tb):
    return np.matmul(R.transpose(), (tb - t))


def getGripperPos(t, R, tb, Rb):
    # Berechne Greiferposition in lokalen Koordinaten
    j2x = armBaseCords[0] + np.multiply(armjointslen[0], m.cos(1.57 + jointSensors[1].getValue()))
    j2y = armBaseCords[2] + np.multiply(armjointslen[0], m.sin(1.57 + jointSensors[1].getValue()))

    j3x = j2x + np.multiply(armjointslen[1], m.cos(1.57 + jointSensors[2].getValue() + jointSensors[1].getValue()))
    j3y = j2y + np.multiply(armjointslen[1], m.sin(1.57 + jointSensors[2].getValue() + jointSensors[1].getValue()))

    px = j3x + np.multiply(armjointslen[2], m.cos(
        1.57 + jointSensors[3].getValue() + jointSensors[2].getValue() + jointSensors[1].getValue()))
    py = j3y + np.multiply(armjointslen[2], m.sin(
        1.57 + jointSensors[3].getValue() + jointSensors[2].getValue() + jointSensors[1].getValue()))

    return np.array([px, 0, py]).reshape(3, 1)


def pickup_box(t, R, tb, Rb):
    gripperpos = getGripperPos(t, R, tb, Rb)

    # Berechne Boxposition in lokalen Koordinaten
    lokalboxpos = getLokalBoxPos(t, R, tb)

    # stamm Motor , 1 gelenk, 2 gelenk, 3 gelenk, spitze dreher Motor
    grasp = [0, -1.13, -0.99, -1.03, 0]

    moveArm(grasp)
    # Wenn arm in Position und Box zwischen Greifern
    if (reachedArm(grasp) and np.linalg.norm(gripperpos - lokalboxpos) < 0.014):
        return True

    return False


picked = False
state = 1
# Fuehre die Simulation aus
while robot.step(timestep) != -1:
    # globale Position und Rotation der roten Zielkiste (Target)
    tt = np.array(target.getPosition()).reshape(3, 1)
    Rt = np.array(target.getOrientation()).reshape(3, 3)
    # Position und Rotation des Roboters
    t = np.array(gps.getPosition()).reshape(3, 1)
    R = np.array(gps.getOrientation()).reshape(3, 3)
    # position und Rotation der Grünen Box
    tb = np.array(box.getPosition()).reshape(3, 1)
    Rb = np.array(box.getOrientation()).reshape(3, 3)

    # Fahre zur grünen Box
    if (state == 1):
        # Berechne Richtungsvektor
        v = calcVecGreenBox(t, tb, R)
        # Berechne Steuervektor
        v = M.dot(v)
        # Anfahrt bis auf 5mm genau
        print()
        if (np.linalg.norm(v, np.inf) < 0.005):
            stop()
            openFingers()
            state = 2
        else:
            setSpeed(v)

    # Nehme grüne Box
    if (state == 2):
        # greife, falls Box nicht gegriffen
        if (not picked):
            picked = pickup_box(t, R, tb, Rb)
        # Box gegriffen, schließe Finger und gehe in Transportposition
        else:
            closeFingers()
            grasp = [0, -0.35, -1.44, 0, 0]
            moveArm(grasp)
            # wenn in Transportposition, fahre zur Roten Box
            if (reachedArm(grasp)):
                state = 0
                picked = False

    # Fahre zur Roten Box
    if (state == 0):
        # behalte Transportposition bei
        grasp = [0, -0.35, -1.44, 0, 0]
        moveArm(grasp)
        # Berechne Richtungsvektor
        v = calcVecRedBox(t, tt, R)
        # Berechne Steuervektor
        v = M.dot(v)
        # Anfahrt bis auf 5cm genau
        if (np.max(v) > 0.05 or np.max(v) < -0.05):
            setSpeed(v)
            if (check_if_drop(t, R, tb, Rb)):
                state = 1
        else:
            # wenn erreicht, halte an und bringe Arm in Abwurfposition
            stop()
            grasp = [0, -1.13, -0.87, 0, 0]
            moveArm(grasp)
            # wenn Abwurfposition erreicht - fallen lassen
            if reachedArm(grasp):
                openFingers()
                state = 3

                # if (output-1600==0):
    #    output=0
    # if (output == 0):
    #    print("Abstand zu Wuerfelmitte: " + str(round(np.linalg.norm(t-tb)-0.29,3)))
    # output += timestep
