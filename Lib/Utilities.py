import math
import numpy as np

def printer(x,y,z):
    print('X: '+str(round(x*1000))+'\t Y: '+str(round(y*1000))+'\t Z: '+str(round(z*1000)), end='\t  ')

def RMSE(S,Sb):
    return math.sqrt(np.square(np.subtract(S,Sb)).mean())

def acc_compute(T, B, s):
    S=np.array(s,dtype=np.float)
    return T@(S-np.diag(B)@np.ones(S.shape))