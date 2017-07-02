from xbox import Joystick

import time

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
        self.inputDevice.refresh()
        return (self.inputDevice.leftX(),
                self.inputDevice.rightY(),
                self.inputDevice.rightX(),
                self.inputDevice.leftY())

    def run(self):
        while self.enabled:
            data = NavigationData(*self.acquireInput())

            if self.inputDevice.Back():
                self.enabled = False
            
            print(str(data))
            time.sleep(1.0 / self.refreshRate)

        print('Halting remote controller')
                
        
        
def linearStep(x):
    return x

def smoothStep(x):
    return x * x * (3.0 - 2.0 * x)

def main():
    controller = RemoteController(linearStep)
    controller.run()
        
    
if __name__ == '__main__':
    main()
