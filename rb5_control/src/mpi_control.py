from numpy import vsplit
from megapi import MegaPi


MFR = 2     # port for motor front right
MBL = 3     # port for motor back left
MBR = 10    # port for motor back right
MFL = 11    # port for motor front left


class MegaPiController:
    def __init__(self, port='/dev/ttyUSB0', verbose=True):
        self.port = port
        self.bot = MegaPi()
        self.bot.start(port=port)
        self.mfr = MFR  # port for motor front right
        self.mbl = MBL  # port for motor back left
        self.mbr = MBR  # port for motor back right
        self.mfl = MFL  # port for motor front left
        self.verbose = verbose
        if verbose:
            self.printConfiguration()

    
    def printConfiguration(self):
        print('MegaPiController:')
        print("Communication Port:", self.port)
        print("Motor ports: MFR:", MFR, "MBL:", MBL, "MBR:", MBR, "MFL:", MFL)


    def setFourMotors(self, vfl=0, vfr=0, vbl=0, vbr=0):
        if self.verbose:
            print("Set Motors: vfl:", vfl, "vfr:", vfr, "vbl:", vbl, "vbr:", vbr)
        self.bot.motorRun(self.mfl,vfl)
        self.bot.motorRun(self.mfr,vfr)
        self.bot.motorRun(self.mbl,vbl)
        self.bot.motorRun(self.mbr,vbr)


    def carStop(self):
        if self.verbose:
            print("CAR STOP:")
        self.setFourMotors()


    def carStraight(self, speed):
        if self.verbose:
            print("CAR STRAIGHT:")
        self.setFourMotors(-speed, speed, -speed, speed)


    def carRotate(self, speed):
        if self.verbose:
            print("CAR ROTATE:")
        self.setFourMotors(speed, speed, speed, speed)


    def carSlide(self, speed):
        if self.verbose:
            print("CAR SLIDE:")
        self.setFourMotors(speed, speed, -speed, -speed)

    
    def carMixed(self, v_straight, v_rotate, v_slide):
        if self.verbose:
            print("CAR MIXED")
        self.setFourMotors(
            v_rotate-v_straight+v_slide,
            v_rotate+v_straight+v_slide,
            v_rotate-v_straight-v_slide,
            v_rotate+v_straight-v_slide
        )