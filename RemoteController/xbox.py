""" Xbox 360 controller support for Python
Original file made on 11/9/13 by Steven Jacobs
Adapted and fixed on 02/07/17 by Leandro Kieliger


This class module supports reading a connected xbox controller.
It requires that xboxdrv be installed first:

    sudo apt-get install xboxdrv

See http://pingus.seul.org/~grumbel/xboxdrv/ for details on xboxdrv

Example usage:

    import xbox
    joy = xbox.Joystick()         #Initialize joystick
    
    if joy.A():                   #Test state of the A button (1=pressed, 0=not pressed)
        print 'A button pressed'
    x_axis   = joy.leftX()        #X-axis of the left stick (values -1.0 to 1.0)
    (x,y)    = joy.leftStick()    #Returns tuple containing left X and Y axes (values -1.0 to 1.0)
    trigger  = joy.rightTrigger() #Right trigger position (values 0 to 1.0)
    
    joy.close()                   #Cleanup before exit
"""

import subprocess
import os
import select
import time
import fcntl

class Joystick:

    """Initializes the joystick/wireless receiver, launching 'xboxdrv' as a subprocess
    and checking that the wired joystick or wireless receiver is attached.
    The refreshRate determines the maximnum rate at which events are polled from xboxdrv.
    Calling any of the Joystick methods will cause a refresh to occur, if refreshTime has elapsed.
    Routinely call a Joystick method, at least once per second, to avoid overfilling the event buffer.
 
    Usage:
        joy = xbox.Joystick()
    """
    def __init__(self,refreshRate = 30):
        self.proc = subprocess.Popen(['sudo','xboxdrv','--detach-kernel-driver'], stdout=subprocess.PIPE)
        self.pipe = self.proc.stdout

        #TODO: Fix unused connect status
        self.connectStatus = False  #will be set to True once controller is detected and stays on
        self.reading = '0' * 140    #initialize stick readings to all zeros

        found = False
        while not found:
            readable, writeable, exception = select.select([self.pipe],[],[],0)
            if readable:

                response = self.pipe.readline()

                # Hard fail if we see this, so force an error
                if 'error' in str(response).lower():
                    while len(response) > 0:
                        print(str(response))
                        response = self.pipe.readline()
                    break

                # Success if we see the following
                if response[0:12] == 'Press Ctrl-c':
                    found = True
                # If we see 140 char line, we are seeing valid input
                if len(response) == 140:
                    found = True
                    self.connectStatus = True
                    self.reading = response
                    
        # if the controller wasn't found, then halt
        if not found:
            self.close()
            raise IOError('Unable to detect Xbox controller/receiver - Run python as sudo')

    
    def refresh(self):
        """
        The refresh function should be called in order to retrieve the output of the driver.
        It performs a non-blocking read on the pipelined stdout of the driver process. Only
        the most recent output line is considered.

        This approaches fixes the issue where select() was used to test for valid output from
        the driver. It failed to retrieve the latest values from the driver.

        A simple readline in a loop does not work since it is a blocking call. See function below
        to perform a non blocking read.
        
        https://stackoverflow.com/questions/8495794/python-popen-stdout-readline-hangs
        
        """
        response = self._nonBlockRead()

        if response is not None:
            l = len(response)

            # Consider only the most recent output from the driver
            if l > 140:
                response = response[l-140:]

            self.reading = response
            print(str(response))

    def _nonBlockRead(self):
        fd = self.pipe.fileno()
        fl = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
        try:
            return self.pipe.read()
        except:
            return None


    """Return a status of True, when the controller is actively connected.
    Either loss of wireless signal or controller powering off will break connection.  The
    controller inputs will stop updating, so the last readings will remain in effect.  It is
    good practice to only act upon inputs if the controller is connected.  For instance, for
    a robot, stop all motors if "not connected()".
    
    An inital controller input, stick movement or button press, may be required before the connection
    status goes True.  If a connection is lost, the connection will resume automatically when the
    fault is corrected.
    """
    def connected(self):
        return self.connectStatus

    # Left stick X axis value scaled between -1.0 (left) and 1.0 (right) with deadzone tolerance correction
    def leftX(self,deadzone=4000):
        raw = int(self.reading[3:9])
        return self.axisScale(raw,deadzone)

    # Left stick Y axis value scaled between -1.0 (down) and 1.0 (up)
    def leftY(self,deadzone=4000):
        raw = int(self.reading[13:19])
        return self.axisScale(raw,deadzone)

    # Right stick X axis value scaled between -1.0 (left) and 1.0 (right)
    def rightX(self,deadzone=4000):
        raw = int(self.reading[24:30])
        return self.axisScale(raw,deadzone)

    # Right stick Y axis value scaled between -1.0 (down) and 1.0 (up)
    def rightY(self,deadzone=4000):
        raw = int(self.reading[34:40])
        return self.axisScale(raw,deadzone)

    # Scale raw (-32768 to +32767) axis with deadzone correction
    # Deadzone is +/- range of values at which the stick is considered to be centered (ie. 0.0)
    # MaxTiltMargin is the value from which the stick is considered to be at maximum tilt
    def axisScale(self,raw,deadzone=4000,maxTiltMargin=2000):
        if abs(raw) < deadzone:
            return 0.0
        else:            
            if raw < 0:
                raw = max(raw, -32768.0 + maxTiltMargin)
                return (raw + deadzone) / (32768.0 - maxTiltMargin - deadzone)
            else:
                raw = min(raw, 32767.0 - maxTiltMargin)
                return (raw - deadzone) / (32767.0 - maxTiltMargin - deadzone)

    # Dpad Up status - returns 1 (pressed) or 0 (not pressed)
    def dpadUp(self):
        return int(self.reading[45:46])

    # Dpad Down status - returns 1 (pressed) or 0 (not pressed)
    def dpadDown(self):
        return int(self.reading[50:51])

    # Dpad Left status - returns 1 (pressed) or 0 (not pressed)
    def dpadLeft(self):
        return int(self.reading[55:56])

    # Dpad Right status - returns 1 (pressed) or 0 (not pressed)
    def dpadRight(self):
        return int(self.reading[60:61])

    # Back button status - returns 1 (pressed) or 0 (not pressed)
    def Back(self):
        return int(self.reading[68:69])

    # Guide button status - returns 1 (pressed) or 0 (not pressed)
    def Guide(self):
        return int(self.reading[76:77])

    # Start button status - returns 1 (pressed) or 0 (not pressed)
    def Start(self):
        return int(self.reading[84:85])

    # Left Thumbstick button status - returns 1 (pressed) or 0 (not pressed)
    def leftThumbstick(self):
        return int(self.reading[90:91])

    # Right Thumbstick button status - returns 1 (pressed) or 0 (not pressed)
    def rightThumbstick(self):
        return int(self.reading[95:96])

    # A button status - returns 1 (pressed) or 0 (not pressed)
    def A(self):
        return int(self.reading[100:101])

    # B button status - returns 1 (pressed) or 0 (not pressed)
    def B(self):
        return int(self.reading[104:105])

    # X button status - returns 1 (pressed) or 0 (not pressed)
    def X(self):
        return int(self.reading[108:109])

    # Y button status - returns 1 (pressed) or 0 (not pressed)
    def Y(self):
        return int(self.reading[112:113])

    # Left Bumper button status - returns 1 (pressed) or 0 (not pressed)
    def leftBumper(self):
        return int(self.reading[118:119])

    # Right Bumper button status - returns 1 (pressed) or 0 (not pressed)
    def rightBumper(self):
        return int(self.reading[123:124])

    # Left Trigger value scaled between 0.0 to 1.0
    def leftTrigger(self):
        return int(self.reading[129:132]) / 255.0

    # Right trigger value scaled between 0.0 to 1.0
    def rightTrigger(self):
        return int(self.reading[136:139]) / 255.0

    # Returns tuple containing X and Y axis values for Left stick scaled between -1.0 to 1.0
    # Usage:
    #     x,y = joy.leftStick()
    def leftStick(self,deadzone=4000):
        return (self.leftX(deadzone),self.leftY(deadzone))

    # Returns tuple containing X and Y axis values for Right stick scaled between -1.0 to 1.0
    # Usage:
    #     x,y = joy.rightStick()
    def rightStick(self,deadzone=4000):
        return (self.rightX(deadzone),self.rightY(deadzone))

    # Cleanup by ending the xboxdrv subprocess
    def close(self):
        os.system('pkill xboxdrv')
