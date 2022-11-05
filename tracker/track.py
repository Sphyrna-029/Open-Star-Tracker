#!/usr/bin/python
import asyncio
import json
import math
import sys  # argv
from urllib.request import urlopen
from urllib.error import URLError
# import RPi.GPIO as GPIO

import motor_control

### Testing ONLY ###
# optional sim hardware
import sim_hardware.sim_GPIO as GPIO
### End Testing ONLY ###


VERBOSE = True
ULTRA_VERBOSE = False
CONFIG_FILE = r"track-config.json"


################################## Load Config ##################################
trackConfig = None
try:
    with open(CONFIG_FILE) as configfile:
        trackConfig = json.load(configfile)
        configfile.close()
        if VERBOSE:
            print("Config loaded from", CONFIG_FILE)
except OSError as exc:
    raise OSError(f"Config {CONFIG_FILE} not found. Make sure this is in same directory as",
                  sys.argv[0]) from exc
################################## End Load Config ##################################


# Init vars
trackerAzimuth: int | None = None  # Tracker Azimuth in degrees 0 - 360
trackerAltitude: int | None = None  # Tracker Altitude in degrees 0 - 90
UPDATE_DELAY = 0.1  # delay between data updates and rotate commands in seconds
MAX_ROT = 1  # maximum degree rotation interval per update


# Get tracker data from stellarium
def getData():
    '''Returns tracking data from Stellarium in horizontal coordinates.
    (azimuth, altitude)
    '''
    try:
        stellariumResponse = urlopen(trackConfig["stellariumAPI"])
    except URLError as exc:
        raise URLError(f"Failed to access stellarium api: {trackConfig['stellariumAPI']}") from exc

    targetData = json.loads(stellariumResponse.read())

    if ULTRA_VERBOSE:
        print("targetData:", targetData)

    targetData = json.loads(targetData["altAz"])

    x, y, z = float(targetData[0]), float(targetData[1]), float(targetData[2])

    # Convert stellarium bullshit view api data to radians
    altitude = math.asin(z)
    azimuth = math.atan2(y, x)

    # Convert from radians to degrees
    azimuth = -math.degrees(azimuth) + 180
    altitude = math.degrees(altitude)

    return azimuth, altitude


async def main():
    ###################################
    ### Initialize MotorControllers ###
    ###################################
    # pins in the form (step, dir, ms1, ms2)
    aziPins = (trackConfig["AziConf"]["AziStepGPIO"],
               trackConfig["AziConf"]["AziDirGPIO"],
               trackConfig["ms1pin"],
               trackConfig["ms2pin"])

    altPins = (trackConfig["AltConf"]["AltStepGPIO"],
               trackConfig["AltConf"]["AltDirGPIO"],
               trackConfig["ms1pin"],
               trackConfig["ms2pin"])

    # constructor arguments (pins, stepsPerRev, gearRatio, name)
    aziArgs = (aziPins, 200, trackConfig["AziConf"]["GearRatio"], "Azimuth")
    altArgs = (altPins, 200, trackConfig["AltConf"]["GearRatio"], "Altitude")

    # Construct Motor Controllers
    aziMotor = motor_control.MotorController(*aziArgs)
    altMotor = motor_control.MotorController(*altArgs)

    if VERBOSE:
        print("\nInitial state: ")
        aziMotor.debugSettings()
        aziMotor.debugStatus()
        altMotor.debugSettings()
        altMotor.debugStatus()
    ###################################


    ########################
    ### EasyDriver Setup ###
    ########################
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(trackConfig["ms1pin"], GPIO.OUT)
    GPIO.setup(trackConfig["ms2pin"], GPIO.OUT)

    # Azimuth
    GPIO.setup(trackConfig["AziConf"]["AziStepGPIO"], GPIO.OUT)
    GPIO.setup(trackConfig["AziConf"]["AziDirGPIO"], GPIO.OUT)

    # Elevation
    GPIO.setup(trackConfig["AltConf"]["AltStepGPIO"], GPIO.OUT)
    GPIO.setup(trackConfig["AltConf"]["AltDirGPIO"], GPIO.OUT)
    ########################

    # set microstep mode
    aziMotor.mstepMode = altMotor.mstepMode = trackConfig["StepMode"]

    GPIO.printBoard()


    try:
        global trackerAzimuth, trackerAltitude
        # Main update loop
        while True:
            trackerAzimuth, trackerAltitude = getData()

            if ULTRA_VERBOSE:
                print("\n\nNew Target Azimuth: " + str(trackerAzimuth)
                      + "\nNew Target Altitude: " + str(trackerAltitude))

            await asyncio.gather(aziMotor.rotateTo(trackerAzimuth,
                                                   ccLimit=(aziMotor.gearOutDegrees - MAX_ROT),
                                                   cwLimit=(aziMotor.gearOutDegrees + MAX_ROT)),
                                 altMotor.rotateTo(trackerAltitude,
                                                   ccLimit=max((altMotor.gearOutDegrees - MAX_ROT),
                                                               trackConfig["AltConf"]["AltMin"]),
                                                   cwLimit=min((altMotor.gearOutDegrees + MAX_ROT),
                                                               trackConfig["AltConf"]["AltMax"])))

            await asyncio.sleep(UPDATE_DELAY)

    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    asyncio.run(main())
