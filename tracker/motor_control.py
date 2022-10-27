'''Higher-level config and motor control functions'''
import json
import math
from os.path import exists
from time import sleep
from urllib.request import urlopen

import RPi.GPIO as GPIO

# Optional sim hardware for testing
# import sim_hardware.sim_GPIO as GPIO
# from sim_hardware.sim_motor import vMotor


# Module settings
VERBOSE = True
ULTRA_VERBOSE = False
CONFIG_FILE = r"track-config.json"
DATA_SRC = r"http://localhost:8090/api/main/view?coord=altAz"


# MotorController constants
DELAY = 0.001  # Delay between GPIO.output() calls in seconds
MSTEP_MODES: dict[int, tuple[bool, bool]] = {
    1: (GPIO.LOW, GPIO.LOW),
    2: (GPIO.HIGH, GPIO.LOW),
    4: (GPIO.LOW, GPIO.HIGH),
    8: (GPIO.HIGH, GPIO.HIGH)
}

class MotorController:
    _id_count = 1

    def __init__(self, pins: tuple[int, int, int | None, int | None],
                 stepsPerRev: int, gearRatio: float = 1, name: str | None = None):
        '''Create a MotorController object

        `pins`: pin/BCM GPIO numbers (step, dir, ms1, ms2)
        `stepsPerRev`: number of full steps per 1 full revolution
        `gearRatio` (optional): gear ratio to an output gear; defaults to 1
        `name` (optional): will autogenerate name if None provided
        '''
        self._units: int = 0
        self._mstepMode: int = 1
        self._gearRatio: int = gearRatio
        self._dir = 1  # 1 for CW and -1 for CCW

        self.PINS: dict[str, int | None] = {name: pin
                                            for name, pin
                                            in zip(("step", "dir", "ms1", "ms2"), pins)}
        self.STEPS_PER_REV = stepsPerRev
        self.name: str = name if name is not None else ("Motor Controller " + str(MotorController._id_count))

        MotorController._id_count += 1

    def mstepsToDegrees(self, numMsteps) -> float:
        return float((numMsteps * 360) / (self.STEPS_PER_REV * self._mstepMode))

    def degreesToMsteps(self, deg) -> int:
        return int((deg * self.STEPS_PER_REV * self._mstepMode) / 360)
    
    @staticmethod
    def _closestLoopMovement(currPos: int, newPos: int, loopSize: int) -> int:
        '''In a closed loop of positive integers (imagine modulo) of size `loopSize`,
        this will calculate the closest relative movement to reach `newPos` from `currPos`.
        
        Note: Because this movement is relative, a directionally signed value is returned.
        You may need to clamp values that you sum with this.'''
        currPos %= loopSize
        newPos %= loopSize
        if (currPos == newPos): return 0

        upper = lower = newPos
        if newPos < currPos:
            upper += loopSize
        else:
            lower -= loopSize
        
        return (lower - currPos) if ((currPos - lower) < (upper - currPos)) else (upper - currPos)

    @property
    def steps(self) -> int:
        '''Motor's position in full steps. (0 <= steps < steps per rotation)'''
        return self._units // self._mstepMode

    @property
    def msteps(self) -> int:
        '''Motor's position in microsteps. (0 <= msteps < msteps per rotation)'''
        return self._units

    @property
    def degrees(self) -> float:
        '''Motor's position in degrees.'''
        return self.mstepsToDegrees(self._units)
    
    @property
    def gearOutDegrees(self):
        '''Output gear's position in degrees.'''
        return self.mstepsToDegrees(self._units) / self._gearRatio

    @property
    def mstepMode(self) -> int:
        return self._mstepMode
    
    @mstepMode.setter
    def mstepMode(self, newMode):
        try:
            newState = MSTEP_MODES[newMode]
        except KeyError as exc:
            raise KeyError("Error: Invalid mstepMode", newMode) from exc

        GPIO.output((self.PINS["ms1"], self.PINS["ms2"]), newState)
        self._mstepMode = newMode

    def _step(self):
        GPIO.output(self.PINS["step"], GPIO.HIGH)
        sleep(DELAY)
        GPIO.output(self.PINS["step"], GPIO.LOW)
        sleep(DELAY)
        self._units = (self._units + self._dir) % (self.STEPS_PER_REV * self._mstepMode)

    def rotate(self, targDeg: float, ccLimit: float = None, cwLimit: float = None, useGearOut:bool = True) -> bool:
        '''Steps a motor to the target degree position

        `targDeg`: target degree position (0 <= targDeg < 360)
        `ccLimit`: inclusive counterclockwise limit in degrees. defaults to None
        `cwLimit`: inclusive clockwise limit in degrees. defaults to None
        `useGearOut`: whether or not targDeg is in terms of output gear position. defaults to True
        Note: When a limit is None, it will be limitless in that direction.
        Returns True if motor moves, false otherwise.
        '''
        relMsteps = MotorController._closestLoopMovement(self.msteps, self.degreesToMsteps(targDeg), self.STEPS_PER_REV * self._mstepMode)
        
        if not relMsteps:
            return False # no movement

        if VERBOSE:
            print(f"{self.name}: Requested rotation to {targDeg} degrees")

        isCCW = relMsteps < 0

        if VERBOSE:
            print(f"{self.name}: Rotating {relMsteps} steps ({'CC' if isCCW else 'CW'})")

        GPIO.output(self.PINS["dir"], isCCW)
        for _ in range(abs(relMsteps)):
            if ((isCCW and ((ccLimit is None) or not ((self.msteps - 1) <= self.degreesToMsteps(ccLimit))))
                    or (not isCCW and ((cwLimit is None) or not ((self.msteps - 1) >= self.degreesToMsteps(cwLimit))))):
                self._step()
            else:
                print(f"{self.name}: Limit reached. Rotation failed!")
                return False  # no movement

        if VERBOSE:
            print(f"{self.name}: Rotation successful!\n")
            self.debugStatus()

        return True  # successful movement

    def debugSettings(self):
        '''Displays motor settings'''
        print(self.name, "SETTINGS")
        print("Pins:", self.PINS)
        print("Steps per revolution:", self.STEPS_PER_REV)
        print("Microstepping:", ("Off" if (self._mstepMode == 1) else ("1/" + str(self._mstepMode))))
        if self._mstepMode != 1:
            print("Microsteps per revolution:", self.STEPS_PER_REV * self._mstepMode)
        print("Gear ratio:", self._gearRatio)
        print()

    def debugStatus(self):
        '''Displays position and target in (micro)steps and degrees'''
        print(self.name, "STATUS")
        print("Direction:", "CC" if (self._dir < 0) else "CW")
        if self._mstepMode == 1:
            print("Step position:", self.steps, "of", self.STEPS_PER_REV)
        else:
            print(f"Microstep (1/{self._mstepMode}) position:", self._units, "of", self.STEPS_PER_REV * self._mstepMode)
        print("Degree position:", self.degrees)
        print("Output gear degree position:", self.gearOutDegrees)
        print()


def loadConfig():
    '''Loads JSON config'''
    config = None
    if not exists(CONFIG_FILE):
        raise Exception(f"Config {CONFIG_FILE} not found. Make sure this is in your working directory.")

    with open(CONFIG_FILE) as configFile:
        config = json.load(configFile)
        configFile.close()
        if VERBOSE:
            print("Config loaded from", CONFIG_FILE)
    
    return config


def getData():
    '''Get data from Stellarium'''
    try:
        stellariumResponse = urlopen(DATA_SRC)

    except:
        print(f"Failed to access stellarium api: {DATA_SRC}")

    targetData = json.loads(stellariumResponse.read())

    if ULTRA_VERBOSE:
        print("Fetching data from", DATA_SRC)
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


#####################################################
########## Pre-MotorController Functions ############
#####################################################

def rotate(motor: vMotor, targDeg: float, ccLimit: float = None, cwLimit: float = None) -> bool:
    '''Steps a vMotor to the target degree position. **This is NOT for real motors.
    Use the MotorController class for controlling real motors.**

    `targDeg` target degree position (0 <= targDeg < 360)
    `ccLimit` inclusive counterclockwise limit in degrees
    `cwLimit` inclusive clockwise limit in degrees
    Note: limit of value `None` is limitless in that direction
    returns `True` if it moved
    '''
    if not isinstance(motor, vMotor):
        raise TypeError("Error: This rotate() function is for vMotors only. Maybe you meant to use MotorController.rotate() for a real motor")

    targDeg %= 360

    # convert target degrees to relative msteps
    upper = lower = motor.degreesToMsteps(targDeg)
    if (upper == motor.msteps):
        return False  # no movement, checks for 0 msteps
    elif (upper < motor.msteps):
        upper += motor.STEPS_PER_REV * motor._mstepMode
    else:
        lower -= motor.STEPS_PER_REV * motor._mstepMode

    relMsteps = (lower - motor.msteps) if ((motor.msteps - lower) < (upper - motor.msteps)) else (upper - motor.msteps)

    if VERBOSE:
        print(f"{motor.name}: Requested rotation to {targDeg} degrees")

    isCCW = relMsteps < 0

    if VERBOSE:
        print(f"{motor.name}: Rotating {relMsteps} steps ({'CC' if isCCW else 'CW'})")

    GPIO.output(motor.PINS["dir"], isCCW)
    for _ in range(abs(relMsteps)):
        if ((isCCW and ((ccLimit is None) or not ((motor.msteps - 1) <= motor.degreesToMsteps(ccLimit))))
            or (not isCCW and ((cwLimit is None) or not ((motor.msteps - 1) >= motor.degreesToMsteps(cwLimit))))):
            GPIO.output(motor.PINS["step"], GPIO.HIGH)
            sleep(DELAY)
            GPIO.output(motor.PINS["step"], GPIO.LOW)
            sleep(DELAY)
        else:
            print(f"{motor.name}: Limit reached. Rotation failed!")
            return False  # no movement

    if VERBOSE:
        print(f"{motor.name}: Rotation successful!\n")
        motor.debugStatus()

    return True  # successful movement

#####################################################


# Testing main
if __name__ == "__main__":
    ### initialize motors ###
    aziMotor = MotorController((16, 18, 17, 24), 200, name="Azimuth")
    altMotor = MotorController((19, 13, 17, 24), 200, name="Altitude")
    #########################

    ### vMotor alternative ###
    # aziMotor = vMotor((16, 18, 17, 24), 200, name="Azimuth")
    # altMotor = vMotor((19, 13, 17, 24), 200, name="Altitude")

    # GPIO.vPlugIn(aziMotor, (16, 18, 17, 24))
    # GPIO.vPlugIn(altMotor, (19, 13, 17, 24))
    ##########################

    trackConfig = loadConfig()
    
    #EasyDriver
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(trackConfig["ms1pin"], GPIO.OUT)
    GPIO.setup(trackConfig["ms2pin"], GPIO.OUT)

    #Azimuth
    GPIO.setup(trackConfig["AziConf"]["AziStepGPIO"], GPIO.OUT)
    GPIO.setup(trackConfig["AziConf"]["AziDirGPIO"], GPIO.OUT)

    #Elevation
    GPIO.setup(trackConfig["AltConf"]["AltStepGPIO"], GPIO.OUT)
    GPIO.setup(trackConfig["AltConf"]["AltDirGPIO"], GPIO.OUT)

    # Microstepping: 8
    GPIO.output(trackConfig["ms1pin"], GPIO.HIGH)
    GPIO.output(trackConfig["ms2pin"], GPIO.HIGH)

    if VERBOSE:
        print("Initial state: ")
        aziMotor.debugSettings()
        altMotor.debugSettings()
        aziMotor.debugStatus()
        altMotor.debugStatus()


    try:
        while True:
            targAzi, targAlt = getData()

            ### Rotate motors ###
            aziMotor.rotate(targAzi)
            altMotor.rotate(targAlt, ccLimit=0, cwLimit=90)
            #####################

            ## Rotate vMotors ###
            # rotate(aziMotor, targAzi)
            # rotate(altMotor, targAlt, ccLimit=0, cwLimit=90)
            ######################
    finally:
        GPIO.cleanup()
