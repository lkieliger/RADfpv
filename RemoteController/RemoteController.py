from xbox import Joystick

import time
import math

DATA_FP_FORMAT = '{0:.2f}'

class NavigationData(object):

    

    def __init__(self, yaw, pitch, roll, thrust):
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll
        self.thrust = thrust

    def __str__(self):
        return ''.join(['Navigation value deltas: \n',
                       'YAW: ',
                       DATA_FP_FORMAT.format(self.yaw),
                       ' PITCH: ',
                       DATA_FP_FORMAT.format(self.pitch),
                       ' ROLL: ',
                       DATA_FP_FORMAT.format(self.roll),
                       ' THRUST: ',
                       DATA_FP_FORMAT.format(self.thrust)])

    def toBitSequence(self):
        pass

class RemoteController(object):

    def __init__(self, inputFadingCurve, refreshRate = 30):
        self.inputFadingCurve = inputFadingCurve
        self.inputDevice = Joystick()
        self.enabled = True
        self.refreshRate = refreshRate

    def acquireInput(self):
        """
        Refreshes and acquire the last available input from the driver

        The current control is left stick form throttle and yaw control,
        right stick for pitch and roll.
        """
        self.inputDevice.refresh()
        return (self.inputFadingCurve(self.inputDevice.leftX()),
                self.inputFadingCurve(self.inputDevice.rightY()),
                self.inputFadingCurve(self.inputDevice.rightX()),
                self.inputFadingCurve(self.inputDevice.leftY()))

    def run(self):
        while self.enabled:
            data = NavigationData(*self.acquireInput())

            if self.inputDevice.Back():
                self.enabled = False
            
            print(str(data))
            time.sleep(1.0 / self.refreshRate)

        print('Halting remote controller')
                

# Ease functions
def linearStep(x):
    return x

def smoothStep(x1):
    x = abs(x1)
    return math.copysign(x * x * (3.0 - 2.0 * x), x1)

def main():
    controller = RemoteController(linearStep)
    controller.run()
    controller.close()
        
    
if __name__ == '__main__':
    main()
