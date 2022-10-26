#!/usr/bin/python
from os.path import exists
from urllib.request import urlopen
import RPi.GPIO as GPIO, time
import json
import time
import math
import ast


################################## Load Config ##################################
if not exists("track-config.json"):
    print("Config track-config.json not found. Make sure this is in same directory as telescope.py")
    exit()


with open("track-config.json") as configfile:
    trackConfig = json.load(configfile)
    configfile.close()
    print("Config loaded!")
################################## End Load Config ##################################

#Init vars
targetData = ""
trackerAltitude = 0 #Tracker Altitude in degrees 0 - 90
trackerAzimuth = 0 #Tracker Azimuth in degrees 0 - 360
delay = 0.01  # 1 millisecond


#Get data from stellarium
def getData():
    global targetData

    try:
        stellariumResponse = urlopen(trackConfig["stellariumAPI"])

    except:
        print("Failed to access stellarium api: {}".format(trackConfig["stellariumAPI"]))

    targetData = json.loads(stellariumResponse.read())

    #print(targetData)

    targetData = ast.literal_eval(targetData["altAz"])

    x, y, z = float(targetData[0]), float(targetData[1]), float(targetData[2])

    #Convert stellarium bullshit view api data to radians
    altitude = math.asin( z )
    azimuth = math.atan2(y, x)

    #Convert from radians to degrees
    azimuth = (math.degrees(azimuth) * -1) + 180
    altitude = math.degrees(altitude)


    return azimuth, altitude


#EasyDriver
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(trackConfig["ms1pin"], GPIO.OUT)
GPIO.setup(trackConfig["ms2pin"], GPIO.OUT)

#Azimuth
GPIO.setup(trackConfig["AziConf"]["AziStepGPIO"], GPIO.OUT)
GPIO.setup(trackConfig["AziConf"]["AziDirGPIO"], GPIO.OUT)

#Elevation
GPIO.setup(trackConfig["AltConf"]["AltStepGPIO"], GPIO.OUT)
GPIO.setup(trackConfig["AltConf"]["AltDirGPIO"], GPIO.OUT)


'''
===============================
 MS1        MS2        Mode
===============================
Low             Low         Full step
High        Low     Half step
Low             High    Quarter step
High        High        Eighth step
===============================
'''

def step(steps, dir, microsteps, motorpin, dir_pin):
    count = 0

    if dir == "cw":
        GPIO.output(dir_pin, GPIO.LOW)

    elif dir == "cc":
        GPIO.output(dir_pin, GPIO.HIGH)

    if microsteps == 1:
        GPIO.output(trackConfig["ms1pin"], GPIO.LOW)
        GPIO.output(trackConfig["ms2pin"], GPIO.LOW)

    elif microsteps == 2:
        GPIO.output(trackConfig["ms1pin"], GPIO.HIGH)
        GPIO.output(trackConfig["ms2pin"], GPIO.LOW)

    elif microsteps == 4:
        GPIO.output(trackConfig["ms1pin"], GPIO.LOW)
        GPIO.output(trackConfig["ms2pin"], GPIO.HIGH)

    elif microsteps == 8:
        GPIO.output(trackConfig["ms1pin"], GPIO.HIGH)
        GPIO.output(trackConfig["ms2pin"], GPIO.HIGH)

    while count < steps:
        GPIO.output(motorpin, GPIO.HIGH)
        time.sleep(delay)
        GPIO.output(motorpin, GPIO.LOW)
        time.sleep(delay)
        count += 1


#Return the opposite of our current bearing. This lets us determine the fastest path to our destination
def inverseDegree(x):

    return (x + 180) % 360


#Check if our azimuth is within a degree of tolerance of our target
def rangeCheck(current, target, tolerance):
    low = (target - tolerance) % 360
    high = (target + tolerance) % 360

    print(low, current, high)

    return (current - low) % 360 <= (high - low) % 360


#Convert steps to deg taking into account our step mode
def stepsToDeg(x, ratio):
    degPerStep = 360 / ((trackConfig["StepMode"] * 200) * ratio)
    Degrees = x * degPerStep

    return Degrees


#Convert degrees to steps taking into account our step mode
def degToStep(x, ratio):
    Steps = x / (360 / ((trackConfig["StepMode"] * 200) * ratio))

    return round(Steps)


# calculates the shortest relative turn in degrees given current and target azimuth
# current and target azimuth assumed to differ by <360 degrees
def calcShortestTurn(currAzi, targAzi):
    upper = lower = targAzi
    if (targAzi < currAzi):
        upper += 360

    else:
        lower -= 360

    deg = (lower - currAzi) if ((currAzi - lower) < (upper - currAzi)) else (upper - currAzi)

    if deg <= 0:
        dir = "cc"
        #convert to pos
        deg = deg * -1

    else:
        dir = "cw"

    return deg, dir


#Move telescope to azimuth target
def gotoAzi(target):
    global trackerAzimuth

    #Take target and trackerAzimuth and determine degree difference
    degDifference, direction = calcShortestTurn(trackerAzimuth, target)

    targetSteps = degToStep(degDifference, trackConfig["AziConf"]["GearRatio"])
    if targetSteps > 0:
        print("Target Azimuth: " + str(target))
        print("Target Azimuth Steps: " + str(targetSteps))
        print("Target Direction: " + direction)

    for steps in range(targetSteps):
         step(1, direction, trackConfig["StepMode"], trackConfig["AziConf"]["AziStepGPIO"], trackConfig["AziConf"]["AziDirGPIO"])
         time.sleep(0.001)
         print("Azimuth Tolerance: " + str(trackerAzimuth - target) )
         trackerAzimuth = target


#Move telescope to Altitude Target
def gotoAlt(target):
    global trackerAltitude

    #Check if target is below horizon
    if target < trackConfig["AltConf"]["AltMin"]:
        print("Target Alt is out of bounds.")
        return

    #Check if target is greater than straight up (This should never happen unless config defaults have changed.)
    if target > trackConfig["AltConf"]["AltMax"]:
        print("Target Alt is out of bounds.")
        return

    degDifference, direction = calcShortestTurn(trackerAltitude, target)

    targetSteps = degToStep(degDifference, trackConfig["AltConf"]["GearRatio"])
    if targetSteps > 0:
        print("Altitude Target: " + str(target))
        print("Target Altitude Steps: " + str(targetSteps))
        print("Target Direction: " + direction)

    for steps in range(targetSteps):
        step(1, direction, trackConfig["StepMode"], trackConfig["AltConf"]["AltStepGPIO"], trackConfig["AltConf"]["AltDirGPIO"])
        time.sleep(0.001)

        trackerAltitude = target



#Placeholder main loop
while True:
    #Get Azi and Alt from Stellarium in degrees
    azimuth, altitude = getData()
    print("\n\nNew Azimuth: " + str(azimuth) + "  Altitude: " + str(altitude))
    gotoAzi(azimuth)
    gotoAlt(altitude)

    time.sleep(0.1)
