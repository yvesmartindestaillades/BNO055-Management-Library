from math import acos
import sys
import board

sys.path.insert(1,'Lib')
from calibration import *

#DEFINE SENSORS HERE
sensorID_0 = 0
sensorType_0 = "bno055"
sensorID_1 = 1
sensorType_1 = "bno055"

if __name__ == "__main__":
    # Create objects to use in the script
    i2c = board.I2C()
    sen0 = BNO055_I2C(i2c, 0x28)
    sen1 = BNO055_I2C(i2c, 0x29)
    cal0 = Calib(sen0, sensorType_0, sensorID_0)
    cal1 = Calib(sen1, sensorType_1, sensorID_1)
    cal0.axis_remap()
    cal1.axis_remap()


    # SCRIPT STARTS HERE
    [T0,B0] = cal0.calibration('all', 'run', N=100)
    cal0.calibration('all', 'save', T=T0, B=B0)
    #[T1,B1] = cal1.calibration('all', 'run', N=50)
   # cal1.calibration('all', 'save', T=T1, B=B1)

    while 1:
        a0=sen0.acceleration
     #   a1=sen1.acceleration
        ac0 = acc_compute(T0,B0,a0)
     #   ac1 = acc_compute(T1,B1,a1)
        print('Sensor 0:', end='')
        printer(ac0[0],ac0[1],ac0[2])
        print('')
     #   print('Sensor 1:', end='')
     #   printer(ac1[0],ac1[1],ac1[2])
     #   print('')
        time.sleep(1)
        