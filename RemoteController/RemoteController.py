from xbox import Joystick

import time
import math
import struct

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

    def toByteSequence(self):
        """
        Pack the navigation data values into a byte sequence.
        Yaw, pitch, roll and thrust values are coded using a signed
        byte. This allows for values from -128 to +127.

        In normal mode (directions are what a pilot would see if he could sit inside the quadcopter, forward facing)
        
            -positive values for yaw means turning clockwise in the same plane as the ground
             while negative values denote a rotation in the opposite direction.
             
            -positive values for pitch means pointing the nose of the quadcopter downwards
             while negative values denote a rotation in the opposite direction.
             
            -positive values for roll means clockwise rotation along the forward axis
             while negative values denote a rotation in the opposite direction.
             
            -positive values for thrust means a general increase in motor rotational speed
             while negative values are meant for deceleration. Note that this means that
             the motors could actually be running at full speed while the thrust value is at 0.
            
        In acrobatic mode:
            [not yet defined]

        Integer value:
        bytes: [0      1      2      3     ]
               [yaw    pitch  roll   thrust]
        """
        data = (self.yaw, self.pitch, self.roll, self.thrust)

        # Converts the values in [-1, 1] interval into the [-128, 127] interval
        data = tuple(map(lambda x: int(127.5 * x - 0.5), data))

        return struct.pack('4b', *data)

    def _toSignedByte(self):
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
            print(str(data.toByteSequence()))
            time.sleep(1.0 / self.refreshRate)

        print('Halting remote controller')
        self.inputDevice.close()
                

# Ease functions
def linearStep(x):
    return x

def smoothStep(x1):
    x = abs(x1)
    return math.copysign(x * x * (3.0 - 2.0 * x), x1)

def main():
    controller = RemoteController(linearStep)
    controller.run()
        
    
if __name__ == '__main__':
    main()
