import ik
import sys
import time
sys.path.append('../dynamixelAPI/')
from dynamixel import *


setTorque(port,packet)
slowDown(port,packet,500)

for i in range(10,46):
    i12,j13,k14 = ik.inverseKinematics(i)
    writePosition(port,packet,-1,i12,j13,k14,-1)
    time.sleep(1)

releaseTorque(port,packet)
