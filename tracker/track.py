#!/usr/bin/python
from urllib.request import urlopen
from urllib.error import URLError
import json
import math
import sys  # argv
import time  # sleep

REAL_MOTOR = False

if REAL_MOTOR:
    import RPi.GPIO as GPIO
else:
    import sim_hardware.sim_GPIO as GPIO
    from sim_hardware.sim_motor import vMotor
    from motor_control import rotate

from motor_control import MotorController, MSTEP_MODES

VERBOSE = True
ULTRA_VERBOSE = False
CONFIG_FILE = r"track-config.json"

################################## Load Config ##################################
try:
    with open(CONFIG_FILE) as configfile:
        trackConfig = json.load(configfile)
        configfile.close()
        if VERBOSE:
            print("Config loaded from", CONFIG_FILE)
except OSError as exc:
    raise OSError(f"Config {CONFIG_FILE} not found. Make sure this is in same directory as", sys.argv[0]) from exc
################################## End Load Config ##################################

#Init vars
targetData = ""
trackerAltitude = 0 #Tracker Altitude in degrees 0 - 90
trackerAzimuth = 0 #Tracker Azimuth in degrees 0 - 360
GPIO_DELAY = 0.01  # delay between GPIO commands in seconds
UPDATE_DELAY = 0.1  # delay between data updates and rotate commands in seconds


#Get data from stellarium
def getData():
    global targetData

    try:
        stellariumResponse = urlopen(trackConfig["stellariumAPI"])
    except URLError as exc:
        raise URLError(f"Failed to access stellarium api: {trackConfig['stellariumAPI']}") from exc

    targetData = json.loads(stellariumResponse.read())

    if ULTRA_VERBOSE:
        print(targetData)

    targetData = json.loads(targetData["altAz"])

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
===================================
 MS1        MS2        Mode
===================================
Low         Low        Full step
High        Low        Half step
Low         High       Quarter step
High        High       Eighth step
===================================
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
        time.sleep(GPIO_DELAY)
        GPIO.output(motorpin, GPIO.LOW)
        time.sleep(GPIO_DELAY)
        count += 1


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

    deg = (currAzi - lower) if ((currAzi - lower) < (upper - currAzi)) else (upper - currAzi)

    if deg <= 0:
        dir = "cc"

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
        print("Azimuth Tolerance: " + str(trackerAltitude - target) )
        trackerAltitude = target


# pins in the form (step, dir, ms1, ms2)
aziPins = (trackConfig["AziConf"]["AziStepGPIO"],
           trackConfig["AziConf"]["AziDirGPIO"],
           trackConfig["ms1pin"],
           trackConfig["ms2pin"])

altPins = (trackConfig["AltConf"]["AltStepGPIO"],
           trackConfig["AltConf"]["AltDirGPIO"],
           trackConfig["ms1pin"],
           trackConfig["ms2pin"])

if REAL_MOTOR:
    ### initialize motors ###
    aziMotor = MotorController(aziPins, 200, name="Azimuth")
    altMotor = MotorController(altPins, 200, name="Altitude")
else:
    ### vMotor alternative ###
    aziMotor = vMotor(aziPins, 200, name="Azimuth")
    altMotor = vMotor(altPins, 200, name="Altitude")

    GPIO.vPlugIn(aziMotor, aziPins)
    GPIO.vPlugIn(altMotor, altPins)

try:
    #Main update loop
    while True:
        #Get Azi and Alt from Stellarium in degrees
        targAzi, targAlt = getData()
        #print("\n\nNew Azimuth: " + str(targAzi) + "  Altitude: " + str(targAlt))

        if REAL_MOTOR:
            ### Rotate motors ###
            aziMotor.rotate(targAzi)
            altMotor.rotate(targAlt, ccLimit=0, cwLimit=90)
        else:
            ### Rotate vMotors ###
            rotate(aziMotor, targAzi)
            rotate(altMotor, targAlt, ccLimit=0, cwLimit=90)
        
        time.sleep(UPDATE_DELAY)
finally:
    GPIO.cleanup()
