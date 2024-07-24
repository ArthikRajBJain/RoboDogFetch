import os
import sys, tty, termios
from dynamixel_sdk import *

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

# Initialize Dynamixel
def initDynamixel(ttyPort):
    portHandler = PortHandler(ttyPort)
    packetHandler = PacketHandler(2.0)
    portHandler.openPort()
    portHandler.setBaudRate(1000000)
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, 116, 4)
    groupSyncRead = GroupSyncRead(portHandler, packetHandler, 132, 4)
    return portHandler, packetHandler, groupSyncWrite, groupSyncRead


# Enable Dynamixel Torque
def setTorque(portHandler,packetHandler):
    packetHandler.write1ByteTxRx(portHandler, 11, 64, 1)
    packetHandler.write1ByteTxRx(portHandler, 12, 64, 1)
    packetHandler.write1ByteTxRx(portHandler, 13, 64, 1)
    packetHandler.write1ByteTxRx(portHandler, 14, 64, 1)
    packetHandler.write1ByteTxRx(portHandler, 15, 64, 1)

# Write goal position
def writePosition(portHandler,packetHandler,val1,val2,val3,val4,val5):
    if(val1 >= 0):
        packetHandler.write4ByteTxRx(portHandler, 11, 116, val1)
    if(val2 >= 0):
        packetHandler.write4ByteTxRx(portHandler, 12, 116, val2)
    if(val3 >= 0):
        packetHandler.write4ByteTxRx(portHandler, 13, 116, val3)
    if(val4 >= 0):
        packetHandler.write4ByteTxRx(portHandler, 14, 116, val4)
    if(val5 >= 0):
        packetHandler.write4ByteTxRx(portHandler, 15, 116, val5)

# Write goal Angle
def writePositionAngle(portHandler,packetHandler,val1,val2,val3,val4,val5):
    if(val1 >= 0):
        packetHandler.write4ByteTxRx(portHandler, 11, 116, int(val1*651.739492))
    if(val2 >= 0):
        packetHandler.write4ByteTxRx(portHandler, 12, 116, int(val2*651.739492))
    if(val3 >= 0):
        packetHandler.write4ByteTxRx(portHandler, 13, 116, int(val3*651.739492))
    if(val4 >= 0):
        packetHandler.write4ByteTxRx(portHandler, 14, 116, int(val4*651.739492))
    if(val5 >= 0):
        packetHandler.write4ByteTxRx(portHandler, 15, 116, int(val5*651.739492))

# Read present position
def readPosition(portHandler,packetHandler,pos):
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, pos, 132)
    return dxl_present_position

# Read present Current
def readCurrent(portHandler,packetHandler,pos):
    dxl_present_current, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, pos, 126)
    return dxl_present_current

def readAngle(portHandler,packetHandler,pos):
    return readPosition(portHandler,packetHandler,pos)*0.001534355

# Read present position for all motors
def readPositionAll(portHandler,packetHandler):
    pos = []
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 11, 132)
    pos.append(dxl_present_position)
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 12, 132)
    pos.append(dxl_present_position)
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 13, 132)
    pos.append(dxl_present_position)
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 14, 132)
    pos.append(dxl_present_position)
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, 15, 132)
    pos.append(dxl_present_position)
    return pos

# Get Velocity of motor
def getVelocity(portHandler,packetHandler,pos):
    dxl_present_velocity, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, pos, 112)
    return dxl_present_velocity

# Set Velocity of motor (0-32767)
def setVelocity(portHandler,packetHandler,pos,val):
    packetHandler.write4ByteTxRx(portHandler, pos, 112, val)

# Disable Dynamixel Torque
def releaseTorque(portHandler,packetHandler):
    packetHandler.write1ByteTxRx(portHandler, 11, 64, 0)
    packetHandler.write1ByteTxRx(portHandler, 12, 64, 0)
    packetHandler.write1ByteTxRx(portHandler, 13, 64, 0)
    packetHandler.write1ByteTxRx(portHandler, 14, 64, 0)
    packetHandler.write1ByteTxRx(portHandler, 15, 64, 0)

def byteSplitter(val):
    return [DXL_LOBYTE(DXL_LOWORD(val)),DXL_HIBYTE(DXL_LOWORD(val)),DXL_LOBYTE(DXL_HIWORD(val)),DXL_HIBYTE(DXL_HIWORD(val))]

def togetherWrite(groupSyncWrite,val1,val2,val3,val4,val5):
    if(val1 != []):
        groupSyncWrite.addParam(11, val1)
    if(val2 != []):
        groupSyncWrite.addParam(12, val2)
    if(val3 != []):
        groupSyncWrite.addParam(13, val3)
    if(val4 != []):
        groupSyncWrite.addParam(14, val4)
    if(val5 != []):
        groupSyncWrite.addParam(15, val5)
    groupSyncWrite.txPacket()
    groupSyncWrite.clearParam()

# set PID values
def setPID(portHandler,packetHandler,pos,P,I,D):
    if(P >= 0):
        packetHandler.write2ByteTxRx(portHandler, pos, 84, P)
    if(I >= 0):
        packetHandler.write2ByteTxRx(portHandler, pos, 82, I)
    if(D >= 0):
        packetHandler.write2ByteTxRx(portHandler, pos, 80, D)

# get PID values
def getPID(portHandler,packetHandler,pos):
    P, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, pos, 84)
    I, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, pos, 82)
    D, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, pos, 80)
    return [P,I,D]

# Position control Functions
def positionHome(portHandler,packetHandler):
    writePosition(portHandler,packetHandler,2040,1000,3060,880,640)

def positionSuitable(portHandler,packetHandler):
    writePosition(portHandler,packetHandler,2040,1328,2567,2386,-1)

def positionTurnLeft(portHandler,packetHandler):
    writePosition(portHandler,packetHandler,3200,1700,1600,1800,620)

def positionTurnRight(portHandler,packetHandler):
    writePosition(portHandler,packetHandler,900,1700,1600,1800,620)

def positionComfortable(portHandler,packetHandler):
    writePosition(portHandler,packetHandler,2043,2099,2374,2693,621)

def positionOpenArm(portHandler,packetHandler):
    writePosition(portHandler,packetHandler,-1,-1,-1,-1,1032)

def positionCloseArm(portHandler,packetHandler):
    writePosition(portHandler,packetHandler,-1,-1,-1,-1,1550)

def positionZControl(portHandler,packetHandler,val):
    # setVelocity(portHandler,packetHandler,11,10000)
    if(val>=900 and val<=3200):
        writePosition(portHandler,packetHandler,val,-1,-1,-1,-1)

def positionReturnBall(portHandler,packetHandler):
    writePosition(portHandler,packetHandler,2040,1929,2428,1752,-1)

def waitPosition(portHandler,packetHandler,val,percent):
    for i in range(5):
        val[i] = int(val[i] - val[i]*percent)
    curVal = readPositionAll(portHandler,packetHandler)
    while(curVal[0]>=val[0] and curVal[1]>=val[1] and curVal[2]>=val[2] and curVal[3]>=val[3] and curVal[4]>=val[4]):
        curVal = readPositionAll(portHandler,packetHandler)
        print(curVal)

def slowDown(portHandler,packetHandler,speed):
    setVelocity(portHandler,packetHandler,12,speed)
    setVelocity(portHandler,packetHandler,13,speed)
    setVelocity(portHandler,packetHandler,14,speed)
    setVelocity(portHandler,packetHandler,15,speed)

def speedUp(portHandler,packetHandler):
    setVelocity(portHandler,packetHandler,12,0)
    setVelocity(portHandler,packetHandler,13,0)
    setVelocity(portHandler,packetHandler,14,0)
    setVelocity(portHandler,packetHandler,15,0)

def grabMotion(portHandler,packetHandler):
    slowDown(portHandler,packetHandler,5000)

# Close port
def closeDynamixel(portHandler):
    portHandler.closePort()

port, packet, groupWrite, groupRead = initDynamixel('/dev/ttyUSB0')
setPID(port,packet,11,400,0,0)
