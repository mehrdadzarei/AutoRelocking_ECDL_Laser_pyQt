######################################################################################################
# @author Mehrdad Zarei <mzarei@umk.pl>
# @date 2021.07.20
# @version 0
#
# @brief Python wrapper for wlmData.dll
#
######################################################################################################


import ctypes
from wlmConst import *
from time import sleep
import numpy as np



class WavelengthMeter:

    def __init__(self, dllpath = "C:\Windows\System32\wlmData.dll", WLM = 'W6', action = 'show'):

        """
        Wavelength Meter class.
        Argument: Optional path to the dll. Default: "C:\Windows\System32\wlmData.dll"
        """

        if WLM == 'W6':
            self.ver = 491
        elif WLM == 'W7':
            self.ver = 4354
        else:
            self.ver = 491
        
        ## WLM Control Mode Constants
        # cCtrlWLMShow = 1
        # cCtrlWLMHide = 2
        # cCtrlWLMExit = 3
        # cCtrlWLMStore = 4
        # cCtrlWLMCompare = 5
        # cCtrlWLMWait        = 0x0010
        # cCtrlWLMStartSilent = 0x0020
        # cCtrlWLMSilent      = 0x0040
        # cCtrlWLMStartDelay  = 0x0080

        self.dll = ctypes.WinDLL(dllpath)

        self.dll.ControlWLMEx.restype = ctypes.c_long
        self.dll.GetOperationState.restype = ctypes.c_ushort
        self.dll.Operation.restype = ctypes.c_long
        self.dll.GetSwitcherChannel.restype = ctypes.c_long
        self.dll.SetSwitcherChannel.restype = ctypes.c_long
        self.dll.GetWavelengthNum.restype = ctypes.c_double
        self.dll.GetFrequencyNum.restype = ctypes.c_double
        self.dll.GetExposureNum.restype = ctypes.c_long
        self.dll.SetExposureNum.restype = ctypes.c_long
        self.dll.GetExposureModeNum.restype = ctypes.c_long
        self.dll.SetExposureModeNum.restype = ctypes.c_long
        self.dll.GetExposureRange.restype = ctypes.c_long
        self.dll.GetPatternItemSize.restype = ctypes.c_long
        self.dll.GetPatternItemCount.restype = ctypes.c_long
        self.dll.SetPattern.restype = ctypes.c_long
        self.dll.GetPatternNum.restype = ctypes.POINTER(ctypes.c_ulong)
        # self.dll.SetWideMode.restype = ctypes.c_long
        # self.dll.GetPatternDataNum.restype = ctypes.c_long
        self.dll.GetAmplitudeNum.restype = ctypes.c_long

        self.run(action)

    def run(self, action):

        if action == 'show':

            self.action = cCtrlWLMShow + cCtrlWLMSilent + cCtrlWLMWait
        elif action == 'hide':

            self.action = cCtrlWLMHide + cCtrlWLMSilent + cCtrlWLMWait

        DATATYPE_MAP = {2: ctypes.c_int, 4: ctypes.c_long, 8: ctypes.c_double}
        self.index = cSignal1WideInterferometer     # cSignal1Interferometers

        # run High Finesse Wavelength Meter
        self.res = self.dll.ControlWLMEx(self.action, 0, ctypes.c_long(self.ver), 10000, 0)

        # check status
        if self.res == flErrUnknownError:

            print('Unknown Error')

        if self.res == flErrTemperatureError:

            print('Temperature Error')

        if self.res == flErrUnknownSN:

            print('Unknown Serial Number')

        if self.res == flErrUnknownDeviceError:

            print('Unknown Device Error')

        if self.res == flErrUSBError:

            print('USB Error')

        if self.res == flErrDriverError:

            print('Driver Error')

        if self.res == flErrDeviceNotFound:

            print('Device Not Found')

        if self.res == flServerStarted:

            print('High Finesse Wavelength Meter Started Successfully!')
            
            self.ch = self.getSwitcherChannel()
            self.dll.SetPattern(self.index, cPatternEnable)
            size = self.dll.GetPatternItemSize(self.index)                              # the size dosen't change for pattern
            self.count = self.dll.GetPatternItemCount(self.index)                       # the count dosen't change for pattern
            self.access_size = DATATYPE_MAP[size]*self.count
            self.address = self.dll.GetPatternNum(ctypes.c_long(self.ch), self.index)   # the address dosen't change for a specific channel
    
    def measurement(self, state):

        # Control Operation
        if self.dll.GetOperationState() != state:
    
            err = self.dll.Operation(state)
            if err == ResERR_WlmMissing:
                self.run('show')
            sleep(2)

            return err

    def getSwitcherChannel(self):

        return self.dll.GetSwitcherChannel()

    def switcherChannel(self, channel, t = 0.3):

        self.ch = channel
        if self.getSwitcherChannel() != self.ch:

            self.dll.SetSwitcherChannel(ctypes.c_long(self.ch))
            sleep(t)     # limitation
            self.address = self.dll.GetPatternNum(ctypes.c_long(self.ch), self.index)

    def getWavelength(self, channel = 0, t = 0.3):

        if channel != 0:

            self.switcherChannel(channel, t)

        return self.dll.GetWavelengthNum(ctypes.c_long(self.ch), ctypes.c_double(0))

    def getFrequency(self, channel = 0, t = 0.3):

        if channel != 0:

            self.switcherChannel(channel, t)
        
        return self.dll.GetFrequencyNum(ctypes.c_long(self.ch), ctypes.c_double(0))

    def getExposure(self, channel = 1, mode = 1):

        # mode 1 for interferometer
        # mode 2 for wide interferometer
        return self.dll.GetExposureNum(ctypes.c_long(channel), ctypes.c_long(mode), 0)

    def setExposure(self, channel = 1, mode = 1, val = 2):

        # mode 1 for interferometer
        # mode 2 for wide interferometer
        err = self.dll.SetExposureNum(ctypes.c_long(channel), ctypes.c_long(mode), ctypes.c_long(val))
        if err == ResERR_CouldNotSet or err == ResERR_ParmOutOfRange:
            self.dll.SetExposureNum(ctypes.c_long(channel), ctypes.c_long(mode), ctypes.c_long(2))

    def getExposureMode(self, channel = 1):

        return self.dll.GetExposureModeNum(ctypes.c_long(channel), 0)

    def setExposureMode(self, channel = 1, val = True):

        self.dll.SetExposureModeNum(ctypes.c_long(channel), ctypes.c_bool(val))

    def getExposureRange(self, ER = cExpoMin):

        # ER cExpoMin
        # ER cExpoMax
        # ER cExpo2Min
        # ER cExpo2Max
        return self.dll.GetExposureRange(ctypes.c_long(ER))

    def spectrum(self, channel = 0, t = 0.3):

        if channel != 0:

            self.switcherChannel(channel, t)
        
        memory_values = ctypes.cast(self.address, ctypes.POINTER(self.access_size))
        
        spectrum_list = []
        n = round(self.count/2)
        for i in range(0, n):
            spectrum_list.append(memory_values.contents[i])
        
        max_main = self.dll.GetAmplitudeNum(ctypes.c_long(self.ch), cMax2)
        max_spec = max(spectrum_list)
        if max_main == 0:

            max_main = max_spec = 1
        
        try:
            ratio = max_spec/max_main
            spectrum_list = np.divide(spectrum_list, ratio)
            spectrum_list = np.interp(np.linspace(0, n, self.count), np.arange(n), spectrum_list)
        except :
            pass

        return spectrum_list

    def exit(self):

        self.measurement(cCtrlStopAll)
        self.dll.ControlWLMEx(cCtrlWLMExit, 0, ctypes.c_long(self.ver), 10000, 0)

