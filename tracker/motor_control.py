'''Provides the MotorController class for high-level motor control'''
import asyncio
sleep = asyncio.sleep

import RPi.GPIO as GPIO

# import sim_hardware.sim_GPIO as GPIO  # Optional sim hardware for testing
# from sim_hardware.sim_GPIO import hiResASleep as sleep  # high res async sleep for Windows testing


# Module settings
_VERBOSE = True

def closestLoopDiff(a: int, b: int, loopSize: int) -> int:
    '''In a "loop" of positive integers (think modulo) of size `loopSize`,
    this will calculate the closest relative difference from `a` to `b`.

    Note: A directionally signed value is returned.'''
    a %= loopSize
    b %= loopSize
    if (a == b):
        return 0

    upper = lower = b
    if b < a:
        upper += loopSize
    else:
        lower -= loopSize

    return int((lower - a) if ((a - lower) < (upper - a)) else (upper - a))


# MotorController constants
GPIO_DELAY = 0.0001  # Delay between GPIO.output() calls in seconds

# =======================================
#  MS1        MS2        Mode
# =======================================
# Low         Low        [1] Full step
# High        Low        [2] Half step
# Low         High       [4] Quarter step
# High        High       [8] Eighth step
# =======================================
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
        `stepsPerRev`: number of full steps per 1 full revolution of the motor
        `gearRatio` (optional): gear ratio to an output gear; defaults to 1
        `name` (optional): will autogenerate name if None provided
        '''
        self._units: int = 0
        self._mstepMode: int = 1
        self._gearRatio: float = gearRatio
        self._dir = 1  # 1 for CW and -1 for CCW

        self.PINS: dict[str, int | None] = {name: pin
                                            for name, pin
                                            in zip(("step", "dir", "ms1", "ms2"), pins)}
        self.STEPS_PER_REV = stepsPerRev
        self.name: str = name if name is not None else ("Motor Controller " + str(MotorController._id_count))

        MotorController._id_count += 1

    def mstepsToDegrees(self, msteps: int, useGearOut: bool = False) -> float:
        '''Convert microsteps to degrees

        `msteps`: msteps to convert
        `useGearOut`: whether or not `msteps` is in terms of output gear position or motor position.
        defaults to False (motor position)
        '''
        if useGearOut:
            return float((msteps * 360) / (self.STEPS_PER_REV * self._mstepMode * self._gearRatio))
        return float((msteps * 360) / (self.STEPS_PER_REV * self._mstepMode))

    def degreesToMsteps(self, deg: float, useGearOut: bool = False) -> int:
        '''Convert degrees to microsteps

        `deg`: degrees to convert
        `useGearOut`: whether or not `deg` is in terms of output gear position or motor position.
        defaults to False (motor position)'''
        if useGearOut:
            return int((deg * self.STEPS_PER_REV * self._mstepMode * self._gearRatio) / 360)
        return int((deg * self.STEPS_PER_REV * self._mstepMode) / 360)

    @property
    def steps(self) -> int:
        '''Motor's position in full steps. (0 <= steps < steps per rotation)'''
        return int(self._units / self._mstepMode)

    @property
    def msteps(self) -> int:
        '''Motor's position in microsteps. (0 <= msteps < msteps per rotation)'''
        return self._units

    @property
    def degrees(self) -> float:
        '''Motor's position in degrees.'''
        return float(self.mstepsToDegrees(self._units))

    @property
    def gearOutDegrees(self) -> float:
        '''Output gear's position in degrees.'''
        return float(self.mstepsToDegrees(self._units, useGearOut=True))

    @property
    def mstepMode(self) -> int:
        return self._mstepMode

    @mstepMode.setter
    def mstepMode(self, newMode: int):
        try:
            newState = MSTEP_MODES[newMode]
        except KeyError as exc:
            raise KeyError("Error: Invalid mstepMode", newMode) from exc

        GPIO.output((self.PINS["ms1"], self.PINS["ms2"]), newState)

        # recalibrate to new mode resolution (assumes closest step of new mode)
        self._units += closestLoopDiff(self._units,
                                           self._units + (newMode - (self._units % newMode)),
                                           newMode)
        self._units = int((self._units * newMode) / self._mstepMode)  # convert to new mode units
        self._mstepMode = newMode

    @property
    def direction(self) -> str:
        '''Motor's direction as "CC" or "CW"'''
        return "CC" if self._dir < 0 else "CW"

    @direction.setter
    def direction(self, newDir: str):
        if newDir == self._dir:
            return

        if newDir.upper() == "CW":
            GPIO.output(self.PINS["dir"], GPIO.LOW)
            self._dir = 1
        elif newDir.upper() == "CC":
            GPIO.output(self.PINS["dir"], GPIO.HIGH)
            self._dir = -1
        else:
            raise ValueError('Error: direction must be either "CW" or "CC" (case insensitive)')

    async def step(self):
        '''Step the motor one step/mstep in current direction with current microstep mode'''
        GPIO.output(self.PINS["step"], GPIO.HIGH)
        await sleep(GPIO_DELAY)
        GPIO.output(self.PINS["step"], GPIO.LOW)
        self._units = (self._units + self._dir) % (self.STEPS_PER_REV * self._mstepMode * self._gearRatio)
        await sleep(GPIO_DELAY)

    async def rotateMsteps(self, msteps: int, ccLimitMstep: int | None = None,
                           cwLimitMstep: int | None = None) -> bool:
        '''`async` method: Steps a motor a relative number of msteps in terms of the output gear or motor
        (see `useGearOut`). It will rotate as far as possible as commanded until it hits the limits, if
        defined. When a limit is None, it will be limitless in that direction. Notice that limits are in
        terms of position and NOT relative msteps.

        `msteps`: relative number of msteps to rotate (positive is CW and negative is CC)
        `ccLimit`: inclusive counterclockwise position limit in degrees. defaults to None
        `cwLimit`: inclusive clockwise position limit in degrees. defaults to None
        `useGearOut`: whether or not `deg` is relative to output gear position or motor position.
        defaults to True (gear position)
        Returns True if motor finishes the full requested rotation, False otherwise.
        '''
        isCC = msteps < 0
        newDir = "CC" if isCC else "CW"

        if not msteps:
            return True  # no rotation

        if _VERBOSE:
            print(f"{self.name}: Rotating {msteps}",
                  ("steps" if (self._mstepMode == 1) else "microsteps"))

        self.direction = newDir
        for _ in range(abs(msteps)):
            if ((isCC and ((ccLimitMstep is None) or ((self._units - 1) > ccLimitMstep)))
                    or (not isCC and ((cwLimitMstep is None) or ((self._units + 1) < cwLimitMstep)))):
                await self.step()
            else:
                if _VERBOSE:
                    print(f"{self.name}: Limit reached!\n")
                    self.debugStatus()
                    return False  # stop rotation

        if _VERBOSE:
            print(f"{self.name}: Requested rotation successful!\n")
            self.debugStatus()

        return True

    async def rotate(self, deg: float,  ccLimit: float | None = None,
                     cwLimit: float | None = None, useGearOut: bool = True) -> bool:
        '''`async` method: Steps a motor a relative number of degrees in terms of the output gear or motor
        (see `useGearOut`). It will rotate as far as possible as commanded until it hits the limits, if
        defined. When a limit is None, it will be limitless in that direction. Notice that limits are in
        terms of position and NOT relative degrees.

        `deg`: relative number of degrees to rotate (positive is CW and negative is CC)
        `ccLimit`: inclusive counterclockwise position limit in degrees. defaults to None
        `cwLimit`: inclusive clockwise position limit in degrees. defaults to None
        `useGearOut`: whether or not `deg` is relative to output gear position or motor position.
        defaults to True (gear position)

        Returns True if motor finishes the requested rotation, False otherwise.
        '''
        relMsteps = self.degreesToMsteps(deg, useGearOut)

        if not relMsteps:
            return True  # no rotation

        if _VERBOSE:
            print(f"{self.name}: Requested rotation of {deg} degrees")

        return await self.rotateMsteps(relMsteps,
                                       None if ccLimit is None else self.degreesToMsteps(ccLimit, useGearOut),
                                       None if ccLimit is None else self.degreesToMsteps(cwLimit, useGearOut))

    async def rotateTo(self, targetDeg: float, ccLimit: float | None = None,
                       cwLimit: float | None = None, useGearOut: bool = True) -> bool:
        '''`async` method: Steps a motor toward a degree position in terms of the output gear or motor
        (see `useGearOut`). It will rotate as far as possible towards the target until it hits the
        limits, if defined. When a limit is None, it will be limitless in that direction.

        `targetDeg`: target degree position (0 <= targetDeg < 360)
        `ccLimit`: inclusive counterclockwise position limit in degrees. defaults to None
        `cwLimit`: inclusive clockwise position limit in degrees. defaults to None

        `useGearOut`: whether or not `deg` is relative to output gear position or motor position.
        defaults to True (gear position)

        Returns True if motor finishes the requested rotation, False otherwise.
        '''
        relMsteps = closestLoopDiff(self._units, self.degreesToMsteps(targetDeg, useGearOut),
                                    int(self.STEPS_PER_REV * self._mstepMode * self._gearRatio))

        if not relMsteps:
            return True  # no rotation

        if _VERBOSE:
            print(f"{self.name}: Requested rotation to {targetDeg} degrees")

        return await self.rotateMsteps(relMsteps,
                                       None if ccLimit is None else self.degreesToMsteps(ccLimit, useGearOut),
                                       None if ccLimit is None else self.degreesToMsteps(cwLimit, useGearOut))

    def debugSettings(self):
        '''Displays MotorController settings'''
        print(self.name, "SETTINGS")
        print("Pins:", self.PINS)
        print("Microstepping:", ("Off" if (self._mstepMode == 1) else ("1/" + str(self._mstepMode))))
        stepLabel = "Steps" if (self._mstepMode == 1) else "Microsteps"
        print(f"{stepLabel} per motor rev:", self.STEPS_PER_REV * self._mstepMode)
        print(f"{stepLabel} per gear rev:", self.STEPS_PER_REV * self._mstepMode * self._gearRatio)
        print("Gear ratio:", self._gearRatio)
        print()

    def debugStatus(self):
        '''Displays motor and gear position in (micro)steps and degrees'''
        print(self.name, "STATUS")
        print("Direction:", self.direction)
        stepLabel = "Step" if (self._mstepMode == 1) else f"Microstep (1/{self._mstepMode})"
        print(stepLabel, "motor position:", self._units % (self.STEPS_PER_REV * self._mstepMode),
              "of", self.STEPS_PER_REV * self._mstepMode)
        print(stepLabel, "gear position:", self._units % (self.STEPS_PER_REV * self._mstepMode * self._gearRatio),
              "of", self.STEPS_PER_REV * self._mstepMode * self._gearRatio)
        print("Motor degree position:", self.degrees)
        print("Output gear degree position:", self.gearOutDegrees)
        print()
