# This Python file uses the following encoding: utf-8
from __future__ import print_function
from sr.robot import *
import time
import threading
from math import *

MAX_LOCATE_ATTEMPT = 1
MAX_GRAB_ATTEMPT = 2
MAX_ALIGN_ATTEMPT = 2
MAX_FIND_ATTEMPT = 2
global robotLookAngle

class CustomisedRuggeduino(Ruggeduino):
    def turn_servo(self, ang):
        with self.lock:
            self.command("s" + chr(ang + 93))

class vec2:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other): #both must be vec2
        newx = self.x + other.x
        newy = self.y + other.y
        return vec2(newx, newy)

    def __sub__(self, other): #both must be vec2
        newx = self.x - other.x
        newy = self.y - other.y
        return vec2(newx, newy)

    def __mul__(self, other):
        if type(other).__name__ == "vec2":
            newx = self.x * other.x
            newy = self.y * other.y
            return vec2(newx, newy)
        else:
            newx = self.x * other
            newy = self.y * other
            return vec2(newx, newy)

    def __rmul__(self, other):
        return self.__mul__(other)

    def __neg__(self):
        return vec2(-self.x, -self.y)

    def __truediv__(self, other):
        if type(other).__name__ == "vec2":
            newx = self.x / other.x
            newy = self.y / other.y
            return vec2(newx, newy)
        else:
            newx = self.x / other
            newy = self.y / other
            return vec2(newx, newy)

    def __div__(self, other):
        if type(other).__name__ == "vec2":
            newx = self.x / other.x
            newy = self.y / other.y
            return vec2(newx, newy)
        else:
            newx = self.x / other
            newy = self.y / other
            return vec2(newx, newy)


    def __str__(self):
        return "vec2("+str(self.x)+","+str(self.y)+")"

    def __eq__(self,other):
        return self.x == other.x and self.y == other.y

    def magnitude(self):
        return (self.x**2 + self.y**2)**0.5

    def normalize(self):
        magnitude = self.magnitude()
        if magnitude > 0:
            self.x = self.x / magnitude
            self.y = self.y / magnitude

        return self

class marker_type:
    NULL, ARENA, ROBOT, TOKEN_A, TOKEN_B, TOKEN_C  = range(6)

#positions are in meters
class marker:
    def __init__(self, pos, markerType, code, norm = 0):
        self.position = pos
        self.markerType = markerType
        self.normal = norm
        self.normalDeg = 180 * norm / pi
        self.code = code
        self.lastSeen = 0
        self.lastNormalUpdate = 0
        self.lastNormalAngle = 0
        self.grabbed = False

    def locate(self):
        turnToPos(40, self.position, update=False, checking=False)

        for m in r.see(res=(1280, 800)):
            if m.info.code == self.code:
                self.lastSeen = time.time()
                return True

        lockedSleep(0.5)

        for m in r.see(res=(1280, 800)):
            if m.info.code == self.code:
                self.lastSeen = time.time()
                return True

        for m in r.see(res=(1280, 800)):
            if m.info.code == self.code:
                self.lastSeen = time.time()
                return True

        for m in r.see(res=(1280, 800)):
            if m.info.code == self.code:
                self.lastSeen = time.time()
                return True

        return self.unknownLocate()

    def unknownLocate(self):
        for i in range(MAX_LOCATE_ATTEMPT):
            

            turn_servo(45)
            robotLookAngle += 45
            lockedSleep(0.6)

            visionTable = r.see(res=(1280, 800))
            for m in visionTable:
                if m.info.code == self.code:
                    updateMarkerLocations(visionTable = visionTable)
                    turn_servo(0)
                    robotLookAngle -= 45
                    turnAngle(40, 45)
                    return True

            turn_servo(-45)
            robotLookAngle -= 90
            lockedSleep(0.6)

            visionTable = r.see(res=(1280, 800))
            for m in visionTable:
                if m.info.code == self.code:
                    updateMarkerLocations(visionTable = visionTable)
                    turn_servo(0)
                    robotLookAngle += 45
                    turnAngle(40, -45)
                    return True

            turn_servo(90)
            robotLookAngle += 135
            lockedSleep(0.6)

            visionTable = r.see(res=(1280, 800))
            for m in visionTable:
                if m.info.code == self.code:
                    updateMarkerLocations(visionTable = visionTable)
                    turn_servo(0)
                    robotLookAngle -= 90
                    turnAngle(40, 90)
                    return True


            turn_servo(-90)
            robotLookAngle -= 180
            lockedSleep(0.6)

            visionTable = r.see(res=(1280, 800))
            for m in visionTable:
                if m.info.code == self.code:
                    updateMarkerLocations(visionTable = visionTable)
                    turn_servo(0)
                    robotLookAngle += 90
                    turnAngle(40, -90)
                    return True

            turn_servo(0)
            robotLookAngle += 90
            turnAngle(40, 180)

            lockedSleep(0.6)

            visionTable = r.see(res=(1280, 800))
            for m in visionTable:
                if m.info.code == self.code:
                    updateMarkerLocations(visionTable = visionTable)
                    return True

        markers[self.code] = marker(vec2(-10, -10), marker_type.NULL, self.code)
        return False

    def align(self, attemptCount=0):
        print("Attempting align:", attemptCount)

        visionTable = sorted(r.see(res=(1280, 800)), key = lambda x: abs(x.orientation.rot_y))

        positionList = []

        lockedSleep(0.5)
        visionTable = sorted(r.see(res=(1280, 800)), key = lambda x: abs(x.orientation.rot_y))

        for m in visionTable:
            if m.info.code == self.code and abs(m.orientation.rot_x) < 50:
                angle = m.orientation.rot_y
                angleRad = pi * angle / 180
                vecAngle = pi * m.rot_y / 180
                markerPos = m.dist * vec2(sin(vecAngle), cos(vecAngle))
                moveVec = markerPos - vec2(sin(angleRad), cos(angleRad)) * 0.7

                positionList.append((moveVec, m.rot_y))

        positionList = sorted(positionList, key = lambda x: x[0].magnitude())

        if len(positionList) > 0:
            moveVec = positionList[0][0]
            ang = positionList[0][1]

            print("-----------------------------")
            print(moveVec)
            print(ang)

            if moveVec.magnitude() > 0.1 and abs(ang) > 5:
                moveByLocalVec(moveVec, 50)
                turnAngle(40, ang / 1.2)
                return self.align()
            elif moveVec.magnitude() > 0.1:
                moveByLocalVec(moveVec, 50)
                return self.align()
            elif abs(ang) > 5:
                turnAngle(40, ang / 1.2)
                return self.align()
            else:
                return True

        moveByLocalVec(vec2(0, -0.8))

        lockedSleep(0.6)
        for m in r.see(res=(1280, 800)):
            if m.info.code == self.code:
                return self.align()

        turn_servo(-45)
        lockedSleep(0.6)
        for m in r.see(res=(1280, 800)):
            if m.info.code == self.code:
                turn_servo(0)
                turnAngle(40, -45)
                return self.align()

        turn_servo(45)
        lockedSleep(0.6)
        for m in r.see(res=(1280, 800)):
            if m.info.code == self.code:
                turn_servo(0)
                turnAngle(40, 45)
                return self.align()

        turn_servo(-90)
        lockedSleep(0.6)
        for m in r.see(res=(1280, 800)):
            if m.info.code == self.code:
                turn_servo(0)
                turnAngle(40, -90)
                return self.align()

        turn_servo(90)
        lockedSleep(0.6)
        for m in r.see(res=(1280, 800)):
            if m.info.code == self.code:
                turn_servo(0)
                turnAngle(40, 90)
                return self.align()

        turn_servo(0)
        turnAngle(40, 180)

        if attemptCount < MAX_ALIGN_ATTEMPT - 1:
            self.align(attemptCount + 1)
        else:
            markers[self.code] = marker(vec2(-10, -10), marker_type.NULL, self.code)
            return False

    def grab(self, attemptCount=0):
        print("Attempting grab:", attemptCount)
        if not self.align():
            return False

        beep()
        flash()
        moveByLocalVec(vec2(0, 0.7), 30)
        move(0, 25, 0.1, autoStop = False)

        for i in range(50):
            if r.ruggeduinos[0].digital_read(2)==False:
                r.power.output[OUT_L3]=True
                self.grabbed = True
                grabbedThread = threading.Thread(target=self.maintainGrab)
                grabbedThread.start()
                lockedSleep(0.3)
                stop()
                lockedSleep(0.3)
                moveByLocalVec(vec2(0, -0.7), 20)
                return True
            lockedSleep(0.05)

        moveByLocalVec(vec2(0, -0.7))

        if attemptCount < MAX_GRAB_ATTEMPT - 1:
            return self.grab(attemptCount + 1)
        else:
            markers[self.code] = marker(vec2(-10, -10), marker_type.NULL, self.code)
            return False

    def maintainGrab(self):
        while self.grabbed:
            time.sleep(1)
            if self.grabbed and r.ruggeduinos[0].digital_read(2)==True:
                robotLock.acquire()
                turn_servo(0)
                r.power.output[OUT_L3]=False
                moveByLocalVec(vec2(0, -1.5))
                turn_servo(0)
                self.grab()
                robotLock.release()
                return

    def takeHome(self):
        turnToPos(40, homeEdge)
        moveByLocalVec(vec2(0, (homeEdge - robotPosition).magnitude() * 0.6))
        updatePosition()
        print("TAKING HOME LOCATION")
        print(robotPosition)
        print("TAKING HOME ANGLE")
        print(robotLookAngle)
        if (homeEdge - robotPosition).magnitude() > 1:
            self.takeHome()
        else:
            turnToAngle(40, 180 + startLookDirections[r.zone], tolerance = 30, updatePos = False, checking=False)
            #turnToPos(40, home)
            self.drop()
            #time.sleep(0.2)
            moveByLocalVec(vec2(0, 1.5), 40)
            #time.sleep(0.2)
            moveByLocalVec(vec2(0, -1))
            self.position = home
            #turnAngle(40, 180)

    def drop(self):
        r.power.output[OUT_L3]=False
        self.grabbed = False

class Supervisor:
    def __init__(self):
        self.startTime = time.time()
        self.status = "Initializing"
        self.supervisingThread = threading.Thread(target=self.run)
        self.supervisingThread.start()

    def run(self):
        self.status = "Nominal"
        while True:
            '''print("")
            print("  NooNoo Supervisor  ")
            print("")
            print("Supervisor Status : " + self.status)
            print("Robot Status      : " + robotStatus)
            print("Runtime           : " + str(time.time() - self.startTime)[:4] + "s")
            print("Location          : " + str(robotPosition.x)[:4] + ", " + str(robotPosition.y)[:4])
            print("Angle             : " + str(robotLookAngle)[:5] + "°")
            print("")
            print("")
            print("Arena map")
            map = []
            for x in range(16):
                map.append(["□", "□", "□", "□", "□", "□", "□", "□", "□", "□", "□", "□", "□", "□", "□", "□"])

            map[int(robotPosition.x) * 2][int(robotPosition.y) * 2] = "N"

            for x in range(16):
                for y in range(16):
                    print(map[15-y][x], end="")

                print("")'''

            time.sleep(0.5)




def lockedSleep(t):
    robotLock.acquire()
    time.sleep(t)
    robotLock.release()

def stop():
    mfl.power = 0
    mfr.power = 0
    mbl.power = 0
    mbr.power = 0

def sign(num):
    if num==0:
        return 0
    if num<0:
        return -1
    return 1

def move(direction, speed, t, degrees = True, autoStop = True):
    if degrees:
        direction = pi * direction / 180

    direction = direction % (2*pi)

    mflpower = cos(direction + pi*0.25)
    mbrpower = cos(direction + pi*0.25)
    mfrpower = sin(direction + pi*0.25)
    mblpower = sin(direction + pi*0.25)

    normalisationFactor = max(abs(mflpower), abs(mfrpower), abs(mblpower), abs(mbrpower))

    mfl.power = (speed * mflpower / normalisationFactor) + 10*sign(mflpower)
    mfr.power = (speed * mfrpower / normalisationFactor) + 10*sign(mfrpower)
    mbl.power = (speed * mblpower / normalisationFactor) + 10*sign(mblpower)
    mbr.power = (speed * mbrpower / normalisationFactor) + 10*sign(mbrpower)

    '''rotationFactor = (mflpower + mblpower - mfrpower - mbrpower) / 4

    mfl.power = (speed * (mflpower - rotationFactor)) + 10*sign(mflpower)
    mfr.power = (speed * (mfrpower - rotationFactor)) + 10*sign(mfrpower)
    mbl.power = (speed * (mblpower + rotationFactor)) + 10*sign(mblpower)
    mbr.power = (speed * (mbrpower + rotationFactor)) + 10*sign(mbrpower)'''


    lockedSleep(t)

    if autoStop:
        stop()

def turn(speed, t, autoStop = True):
    mfl.power = speed
    mbl.power = speed
    mfr.power = -speed
    mbr.power = -speed

    lockedSleep(t)
    if autoStop:
        stop()

def turnAngle(speed, angle, degrees = True, autoStop = True):
    if not degrees:
        angle = 180 * angle / pi

    if angle > 180:
        angle = -(360 - angle)

    if angle < -180:
        angle = 360 + angle

    if abs(angle) > 1:
        turn((speed + 10) * sign(angle), abs(angle / speed) * 0.5, autoStop)

def turnToAngle(speed, angle, tolerance = 30, updatePos = True, checking=True):
    lockedSleep(0.5)
    if updatePos:
        updatePosition()


    print("LOOK ANGLE")
    print(angle)
    print("ROBOT LOOK ANGLE")
    print(robotLookAngle)

    ang = (angle%360) - robotLookAngle

    if ang > 180:
        ang = -(360 - ang)

    if ang < -180:
        ang = 360 + ang


    print("WILL TURN")
    print(ang)
    if abs(ang) > tolerance:
        if checking:
            turnAngle(speed, ang / 1.5)
            turnToAngle(speed, angle, tolerance)
        else:
            turnAngle(speed, ang)

def turnToPos(speed, pos, tolerance = 30, update=True, checking=True):
    if update:
        updatePosition()

    print("TURNING TO")
    print(pos)
    print("ROBOT POS")
    print(robotPosition)

    lookVec = pos - robotPosition
    lookAng = 180 * atan2(lookVec.x, lookVec.y) / pi
    turnToAngle(speed, lookAng, tolerance, False, checking)

def getClosestMarker(markerType, updateMarkers = True):
    if updateMarkers:
        updateMarkerLocations()

    closest = None
    closestDist = 100
    for marker in markers:
        if marker.markerType == markerType and (marker.position - home).magnitude() > 0.75:
            distance = (marker.position - robotPosition).magnitude()
            if distance < closestDist:
                closest = marker
                closestDist = distance

    return closest

def updatePosition(force = True):

    visionTable = sorted(r.see(res=(1280, 800)), key = lambda x: abs(x.dist))

    mNumber = 0
    totalPos = vec2(0,0)

    for m in visionTable:
        marker = markers[m.info.code]
        if marker != False and marker.markerType == marker_type.ARENA:
            rotation = pi * m.orientation.rot_y / 180
            offset = vec2(sin(-rotation + marker.normal),
                        cos(-rotation + marker.normal))

            offset *= m.centre.polar.length

            global robotPosition
            global robotLookAngle

            mNumber += 1
            totalPos += marker.position - offset
            robotLookAngle = (-m.orientation.rot_y - m.centre.polar.rot_y + marker.normalDeg)%360

            updateMarkerLocations(visionTable = visionTable)

    if mNumber > 0:
        robotPosition = totalPos / mNumber

        return True


    lockedSleep(0.5)

    visionTable = sorted(r.see(res=(1280, 800)), key = lambda x: abs(x.dist))

    mNumber = 0
    totalPos = vec2(0,0)

    for m in visionTable:
        marker = markers[m.info.code]
        if marker != False and marker.markerType == marker_type.ARENA:
            rotation = pi * m.orientation.rot_y / 180
            offset = vec2(sin(-rotation + marker.normal),
                        cos(-rotation + marker.normal))

            offset *= m.centre.polar.length

            global robotPosition
            global robotLookAngle

            mNumber += 1
            totalPos += marker.position - offset
            robotLookAngle = (-m.orientation.rot_y - m.centre.polar.rot_y + marker.normalDeg)%360

            updateMarkerLocations(visionTable = visionTable)

    if mNumber > 0:
        robotPosition = totalPos / mNumber

        return True

    turn_servo(45)
    lockedSleep(0.5)

    visionTable = sorted(r.see(res=(1280, 800)), key = lambda x: abs(x.dist))

    mNumber = 0
    totalPos = vec2(0,0)

    for m in visionTable:
        marker = markers[m.info.code]
        if marker!= False and marker.markerType == marker_type.ARENA:
            rotation = pi * m.orientation.rot_y / 180
            offset = vec2(sin(-rotation + marker.normal),
                        cos(-rotation + marker.normal))

            offset *= m.centre.polar.length

            global robotPosition
            global robotLookAngle

            mNumber += 1
            totalPos += marker.position - offset
            robotLookAngle = (-m.orientation.rot_y - m.centre.polar.rot_y + marker.normalDeg - 45)%360

    turn_servo(0)

    if mNumber > 0:
        robotPosition = totalPos / mNumber

        return True


    turn_servo(-45)
    lockedSleep(0.5)

    visionTable = sorted(r.see(res=(1280, 800)), key = lambda x: abs(x.dist))

    mNumber = 0
    totalPos = vec2(0,0)

    for m in visionTable:
        marker = markers[m.info.code]
        if marker != False and marker.markerType == marker_type.ARENA:
            rotation = pi * m.orientation.rot_y / 180
            offset = vec2(sin(-rotation + marker.normal),
                        cos(-rotation + marker.normal))

            offset *= m.centre.polar.length

            global robotPosition
            global robotLookAngle

            mNumber += 1
            totalPos += marker.position - offset
            robotLookAngle = (-m.orientation.rot_y - m.centre.polar.rot_y + marker.normalDeg + 45)%360

    turn_servo(0)

    if mNumber > 0:
        robotPosition = totalPos / mNumber

        return True

    turn_servo(90)
    lockedSleep(0.7)

    visionTable = sorted(r.see(res=(1280, 800)), key = lambda x: abs(x.dist))

    mNumber = 0
    totalPos = vec2(0,0)

    for m in visionTable:
        marker = markers[m.info.code]
        if marker != False and marker.markerType == marker_type.ARENA:
            rotation = pi * m.orientation.rot_y / 180
            offset = vec2(sin(-rotation + marker.normal),
                        cos(-rotation + marker.normal))

            offset *= m.centre.polar.length

            global robotPosition
            global robotLookAngle

            mNumber += 1
            totalPos += marker.position - offset
            robotLookAngle = (-m.orientation.rot_y - m.centre.polar.rot_y + marker.normalDeg - 90)%360

    turn_servo(0)

    if mNumber > 0:
        robotPosition = totalPos / mNumber

        return True

    turn_servo(-90)
    lockedSleep(0.9)

    visionTable = sorted(r.see(res=(1280, 800)), key = lambda x: abs(1/x.dist))

    mNumber = 0
    totalPos = vec2(0,0)

    for m in visionTable:
        marker = markers[m.info.code]
        if marker != False and marker.markerType == marker_type.ARENA:
            rotation = pi * m.orientation.rot_y / 180
            offset = vec2(sin(-rotation + marker.normal),
                        cos(-rotation + marker.normal))

            offset *= m.centre.polar.length

            global robotPosition
            global robotLookAngle

            mNumber += 1
            totalPos += marker.position - offset
            robotLookAngle = (-m.orientation.rot_y - m.centre.polar.rot_y + marker.normalDeg + 90)%360

    turn_servo(0)

    if mNumber > 0:
        robotPosition = totalPos / mNumber

        return True

    turn_servo(0)
    if force:
        turnAngle(40, 180)
        return updatePosition()

def moveTo(pos, endAngle = 0, forcePositionUpdate = True, tolerance=0.3):
    updated = updatePosition()
    if forcePositionUpdate and not updated:
        while not updated:
            updated = updatePosition()
            stop()
            lockedSleep(0.01)

    print("CURRENT POSITION")
    print(robotPosition)
    print("GOING TO")
    print(pos)

    moveVec = pos - robotPosition
    moveDir = 180 * atan2(moveVec.x, moveVec.y) / pi #use xy to flip axis (ang from y)
    moveDir -= robotLookAngle

    print("MOVE VEC")
    print(moveVec)
    print(moveVec.magnitude())
    print(tolerance)

    speed = max(50 * min(moveVec.magnitude(), 1), 10)

    if moveVec.magnitude() > tolerance:
        move(moveDir, speed, distanceToTime(speed, moveVec.magnitude() * 0.7), degrees = True, autoStop = True)
        lockedSleep(0.1)
        if endAngle != 0:
            turnAngle(40, (endAngle - robotLookAngle), degrees = True, autoStop = True)

        moveTo(pos, endAngle, forcePositionUpdate, tolerance)
    else:
        stop()

def fastForward(speed, time):
    flash()
    mfl.power = speed
    mfr.power = speed
    mbl.power = 30
    mbr.power = 30
    lockedSleep(time)
    flash()
    stop()

def moveByLocalVec(moveVec, maxspeed = 50, minspeed = 10):
    moveDir = 180 * atan2(moveVec.x, moveVec.y) / pi
    speed = max(maxspeed * 2 * min(moveVec.magnitude(), 0.5), minspeed)

    if moveVec.magnitude() > 0.05:
        move(moveDir, speed, distanceToTime(speed, moveVec.magnitude()), degrees = True, autoStop = True)
    else:
        stop()

def updateMarkerLocations(visionTable = False):
    if not visionTable:
        visionTable = r.see(res=(1280, 800))
    for m in visionTable:
        if m.info.marker_type in (MARKER_TOKEN_A, MARKER_TOKEN_B, MARKER_TOKEN_C):
            angle = pi * (robotLookAngle + m.rot_y) / 180
            offset = m.dist * vec2(sin(angle), cos(angle))

            position = robotPosition + offset

            print("FOUND MARKER AT")
            print(position)

            markers[m.info.code].lastSeen = time.time()
            markers[m.info.code].position = position
            if m.info.marker_type == MARKER_TOKEN_A:
                markers[m.info.code].markerType = marker_type.TOKEN_A
            elif m.info.marker_type == MARKER_TOKEN_B:
                markers[m.info.code].markerType = marker_type.TOKEN_B
            else:
                markers[m.info.code].markerType = marker_type.TOKEN_C

def distanceToTime(speed, distance):
    if speed != 0:
        return distance / (2*(speed / 100.0))
    else:
        return 0

def turn_servo(angle, degrees = True):
    if not degrees:
        angle = 180 * angle / pi
    r.ruggeduinos[0].turn_servo(angle)

def beep(t = 200, note ='d'):
    r.power.beep(t, note = note)

def flash():
    t = threading.Thread(target=doFlash)
    t.start()

def doFlash():
    r.power.output[OUT_L1]=False
    time.sleep(0.1)
    r.power.output[OUT_L1]=True
    time.sleep(0.1)
    r.power.output[OUT_L1]=False
    time.sleep(0.1)
    r.power.output[OUT_L1]=True

def awaitStart():
    while awaitingStart:
        flash()
        time.sleep(1)


def getMarkerOfType(t):
    m = None
    testCount = 0
    while testCount < 9:
        m = getClosestMarker(t)
        if not m:
            testCount += 1
            turnAngle(40, 45)
            time.sleep(1)
        else:
            return m

    moveTo(vec2(4, 4))
    updatePosition()


awaitingStart = True
r = Robot.setup()
r.ruggeduino_set_handler_by_fwver("SRcustom", CustomisedRuggeduino)
r.init()
r.power.output[OUT_L3]=False
awaitStartThread = threading.Thread(target=awaitStart)
awaitStartThread.start()

mbr = r.motors[0].m0
mbl = r.motors[0].m1
mfl = r.motors[1].m0
mfr = r.motors[1].m1
r.ruggeduinos[0].pin_mode(2, INPUT_PULLUP)

#in meters
#robotPosition = vec2(0.5, 7.5)
homeEdges = [vec2(1, 7), vec2(7, 7), vec2(7, 1), vec2(1, 1)]
#homeEdge = vec2(1, 7)
homePositions = [vec2(0.5, 7.5), vec2(7.5, 7.5), vec2(7.5, 0.5), vec2(0.5, 0.5)]
startLookDirections = [135, 225, 315, 45]
#home = vec2(0.5, 7.5)
#in degrees
robotLookAngle = 135
servoAngle = 0
robotStatus = "Initializing"
markers = []

markerCodeTemp = 0

robotLock = threading.RLock()
supervisor = Supervisor()


for i in range(1, 8):
    markers.append(marker(vec2(i, 8), marker_type.ARENA, markerCodeTemp))
    markerCodeTemp += 1
for i in range(7, 0, -1):
    markers.append(marker(vec2(8, i), marker_type.ARENA, markerCodeTemp, norm=0.5*pi))
    markerCodeTemp += 1
for i in range(7, 0, -1):
    markers.append(marker(vec2(i, 0), marker_type.ARENA, markerCodeTemp, norm=pi))
    markerCodeTemp += 1
for i in range(1, 8):
    markers.append(marker(vec2(0, i), marker_type.ARENA, markerCodeTemp, norm=1.5*pi))
    markerCodeTemp += 1
for i in range(60):
    markers.append(marker(vec2(-10, -10), marker_type.NULL, markerCodeTemp))
    markerCodeTemp += 1

r.wait_start()
awaitingStart = False
turn_servo(0)

homeEdge = homeEdges[r.zone]
home = homePositions[r.zone]
robotPosition = home
robotLookAngle = startLookDirections[r.zone]

'''while True:
    if r.ruggeduinos[0].digital_read(2)==False:
        r.power.output[OUT_L3]=True
    else:
        r.power.output[OUT_L3]=False

    time.sleep(0.1)'''

updateMarkerLocations()

moveByLocalVec(vec2(0, 1))
updateMarkerLocations()
lockedSleep(0.5)
updateMarkerLocations()

#fastForward(60, 8)
#stop()
#lockedSleep(1)
#moveByLocalVec(vec2(0, -1))



tries = 0
while True and tries < MAX_FIND_ATTEMPT:
    m = getMarkerOfType(marker_type.TOKEN_B)


    tries += 1
    if m and m.locate():
        if m.grab():
            m.takeHome()
            break

tries = 0
while True and tries < MAX_FIND_ATTEMPT:
    m = getMarkerOfType(marker_type.TOKEN_A)


    tries += 1
    if m and m.locate():
        if m.grab():
            m.takeHome()
            break


moveTo(vec2(4, 4), endAngle = 0, forcePositionUpdate = True, tolerance=1)


tries = 0
while True and tries < MAX_FIND_ATTEMPT:
    m = getMarkerOfType(marker_type.TOKEN_C)


    tries += 1
    if m and m.locate():
        if m.grab():
            m.takeHome()
            break


while True:
    while True:
        m = getMarkerOfType(marker_type.TOKEN_B)
        if not m:
            m = getMarkerOfType(marker_type.TOKEN_A)

        if m and m.locate():
            if m.grab():
                m.takeHome()
                break


'''while True:
    updatePosition()
    time.sleep(0.5)'''
