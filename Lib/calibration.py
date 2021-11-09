
"""
`calibration_adafruit_bno055`
=======================================================================================

This is a CircuitPython driver for calibration of the Bosch BNO055 nine degree of freedom
inertial measurement unit module with sensor fusion.

* Author(s): Yves Martin


**Hardware:**

* Adafruit `9-DOF Absolute Orientation IMU Fusion Breakout - BNO055
  <https://www.adafruit.com/product/4646>`_ (Product ID: 4646)


**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

* Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice

* Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register

"""

from ctypes import sizeof
from adafruit_bno055 import *
import time
import numpy as np
from Utilities import *

GRAVITY = 9.80368

class Calib:
    def __init__(self, sensor, sensorType, sensorID):
        self.s = sensor
        self.onboardfile = "onboardCalib_"+sensorType+"#"+str(sensorID)
        self.datafile = "DataCalib_"+sensorType+"#"+str(sensorID)
        self.fullfile = "FullCalib_"+sensorType+"#"+str(sensorID)

    def set_offset(self,o=(0,0,0)):
        self.s.mode = CONFIG_MODE
        self.s.offset_accelerometer= o
        self.s.mode = ACCONLY_MODE

    def data_collect(self, N=100):
        # OUTPUT
        #   S matrix = data manually acquired
        S = np.zeros((3, 6*N),dtype=np.float64)

        faces = ['A','B','C','D','E','F']
        i=0
        while i < 6: 
            print("Show face "+faces[i]+", then press enter! Or type any letter to start over previous face")
            inp = input()
            while(inp != ''):
                if inp:
                    i=i-2
                    break
                time.sleep(0.001)
                inp=input()
            if inp == '':
                print("Measuring the acceleration...")
                j=0
                while j<N:
                    S[:,i*N+j]=self.s.acceleration
                    if math.sqrt(np.sum(np.square(S[:,i*N+j]))) > GRAVITY*1.5:
                        j=j-1
                    else:
                        print('#'*j)
                    j=j+1
                    time.sleep(0.05) 
                print("\nDone! RMS is ", (S[:,i*N:(i+1)*N]).std(axis=1).mean()*1000, end='mN and mean vector is ')
                if i == 5:
                    print('')
                print(np.mean(S[:,i*N:(i+1)*N],axis=1))
            i=i+1
        return S,N

    def data_load(self):
    # Uploads the manual data from a .npy file to the board
        return np.load(self.datafile+".npy")

    def data_save(self,S):
    # Downloards the manual data to a .npy file
        np.save(arr=S,file=self.datafile+".npy")

    def calibration(self, type, instruction, T=False, B=False, N=False, S=False):
    # Generic function ofr calibration
    # Input: 
    #   Instruction: 
    #       load: loads the parameters to the board and returns the offboard parameters if relevant
    #       save: saves the parameters to the appropriate file on the computer 
    #       run : starts the calibration process with the given parameters
    #   Type: 
    #       all: performs the onboard and offboard calibration   
    #       onboard: performs the onboard calibration only
        if type == 'onboard':
            if instruction == 'load':
                return param_load_onboard(self.onboardfile, self.s)
            if instruction == 'save':
                param_save_onboard(self.onboardfile, self.s)         
            if instruction == 'compute':
                self.auto_calib()
        if type == 'all':
            if instruction == 'load':
                return param_load_full_config(self.fullfile, self.s)
            if instruction == 'save':
                if T is not False and B is not False:
                    param_save_full_config(self.fullfile, T, B, self.s)
                else:
                    raise("T or B are missing for saving the configuration")
            if instruction == 'run':
                self.auto_calib()
                if S:
                    if N:
                        return self.param_compute_offboard(S, N)
                    else:
                        return self.param_compute_offboard(S, S.shape[1]/6)
                else:
                    if N:
                        [S,N] =self.data_collect(N)
                    else:
                        [S,N] =self.data_collect()
                    return self.param_compute_offboard(S,N)

    def auto_calib(self):
    # Calibrates the sensor using the autocalib function (=onboard calib)
        self.s.mode = NDOF_MODE
        accCalib, gyrCalib, magCalib, sysCalib = 0, 0, 0, 0
        while(not sysCalib):
            [sysCalibNew, gyrCalibNew, accCalibNew, magCalibNew]= self.s.calibration_status
            if not accCalib and accCalibNew:
                accCalib = accCalibNew
                print('Accelerometer calibrated!')
            if not gyrCalib and gyrCalibNew:
                gyrCalib = gyrCalibNew
                print('Gyrometer calibrated!')
            if not magCalib and magCalibNew:
                magCalib = magCalibNew
                print('Magnetometer calibrated!')
            if not sysCalib and sysCalibNew:
                sysCalib = sysCalibNew
                print('Victory! System calibrated!')
            time.sleep(0.1)

    def param_compute_offboard(self,S, N):
        # OUTPUT
        #   T: basis change matrix + scaling matrix
        #   B: bias matrix
        #   S' = T*(S-B)

        # Create variables for the next steps
        B = np.zeros((1,3),dtype=S.dtype) # Bias of the measurements
        Sb = np.zeros((3,6*N),dtype=S.dtype) # Perfect IMU data
        for i in range(6):
            Sb[int(i/2),i*N:(i+1)*N]=GRAVITY*pow(-1,i)

        # Compute bias
        B = np.mean(S,axis=1) 

        # Compute linear transformation
        Sbsq=Sb@np.transpose(Sb)
        SSbsq  =(S-(np.diag(B)@np.ones(S.shape)))@np.transpose(Sb) 
        T = np.linalg.solve(SSbsq, Sbsq)

        # Compute performances
        RMSE_raw=RMSE(S,Sb)
        RMSE_proc= RMSE(acc_compute(T,B,S),Sb)

        print("RMS error for onboard calibration is :",RMSE_raw)
        print("RMS error for full calib is :", RMSE_proc)
        return T,B
    
    def axis_remap(self):
        self.s.axis_remap = (AXIS_REMAP_Y, AXIS_REMAP_Z, AXIS_REMAP_X,\
             AXIS_REMAP_POSITIVE, AXIS_REMAP_NEGATIVE, AXIS_REMAP_NEGATIVE)
    

def param_load_manual(file):
    # Uploads the calibration from a .npy file to the board
    loaded=np.load(file+".npy")
    T= loaded[0:3,:]
    B= loaded[3,:]
    return T,B

def param_save_manual(T, B, file):
    # Saves the calibration from the board to a .npy file
    formatted_matrix = np.zeros((4,3), dtype=np.float64)
    formatted_matrix[0:3]=T
    formatted_matrix[3]=B
    np.save(arr=formatted_matrix, file=file)

def param_load_onboard(file,s):
    # Loads the calibration from a .npy file to the board
    loaded = np.load(file=file)
    s.offsets_accelerometer=loaded[0]
    s.offsets_magnetometer=loaded[1]
    s.offsets_gyroscope= loaded[2] 

def param_save_onboard(file, s):
    # Saves the calibration from the board to a .npy file
    formatted_matrix = np.zeros((3,3), dtype=np.float64)    
    formatted_matrix[0]=s.offsets_accelerometer
    formatted_matrix[1]=s.offsets_magnetometer
    formatted_matrix[2]=s.offsets_gyroscope
    np.save(arr=formatted_matrix, file=file)

def param_save_full_config(file,T,B,s):
    formatted_matrix = np.zeros((7,3), dtype=np.float64)
    formatted_matrix[0:3]=T
    formatted_matrix[3]=B
    formatted_matrix[4]=s.offsets_accelerometer
    formatted_matrix[5]=s.offsets_magnetometer
    formatted_matrix[6]=s.offsets_gyroscope
    np.save(arr=formatted_matrix, file=file)

def param_load_full_config(file,s):
    loaded=np.load(file+".npy")
    T= loaded[0:3]
    B= loaded[3]
    s.offsets_accelerometer= tuple(loaded[4].astype(np.int))
    s.offsets_magnetometer=  tuple(loaded[5].astype(np.int))
    s.offsets_gyroscope=  tuple(loaded[6].astype(np.int))
    return T,B