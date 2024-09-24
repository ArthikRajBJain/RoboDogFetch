import sys
import time
import math
import robot_interface as sdk


HIGHLEVEL = 0xee
udp = sdk.UDP(HIGHLEVEL, 8080, "192.168.123.161", 8082)
cmd = sdk.HighCmd()
# state = sdk.HighState()
udp.InitCmdData(cmd)

# udp.Recv()
# udp.GetRecv(state)

cmd.mode = 0
cmd.gaitType = 0
cmd.speedLevel = 0
cmd.footRaiseHeight = 0
cmd.bodyHeight = 0
cmd.euler = [0, 0, 0]
cmd.velocity = [0, 0]
cmd.yawSpeed = 0.0
cmd.reserve = 0

# Low Power Sleep
def lowPowerSleep(cmd,udp):
    cmd.mode = 5
    cmd.velocity = [0, 0]
    cmd.gaitType = 0
    cmd.yawSpeed = 0
    udp.SetSend(cmd)
    udp.Send()
    cmd.mode = 7
    udp.SetSend(cmd)
    udp.Send()

# Sleep Flat
def sleepFlat(cmd,udp):
    cmd.mode = 5
    cmd.velocity = [0, 0]
    cmd.yawSpeed = 0
    cmd.gaitType = 0
    udp.SetSend(cmd)
    udp.Send()

# Stand Up
def standUp(cmd,udp):
    cmd.mode = 6
    cmd.velocity = [0, 0]
    cmd.gaitType = 0
    cmd.yawSpeed = 0
    udp.SetSend(cmd)
    udp.Send()
    time.sleep(0.25)
    cmd.mode = 1
    udp.SetSend(cmd)
    udp.Send()

# Move Forward
def moveForward(cmd,udp,quantity):
    cmd.mode = 2
    cmd.velocity = [quantity, 0]
    cmd.gaitType = 1
    cmd.yawSpeed = 0
    udp.SetSend(cmd)
    udp.Send()

# Move Backward
def moveBackward(cmd,udp,quantity):
    cmd.mode = 2
    cmd.velocity = [-1.0*quantity, 0]
    cmd.gaitType = 1
    cmd.yawSpeed = 0
    udp.SetSend(cmd)
    udp.Send()

# Rotate ClockWise
def rotateClockwise(cmd,udp,quantity):
    cmd.mode = 2
    cmd.velocity = [0, 0]
    cmd.yawSpeed = -1.0*quantity
    cmd.gaitType = 1
    udp.SetSend(cmd)
    udp.Send()

# Rotate Counter ClockWise
def rotateCounterClockwise(cmd,udp,quantity):
    cmd.mode = 2
    cmd.velocity = [0, 0]
    cmd.yawSpeed = quantity
    cmd.gaitType = 1
    udp.SetSend(cmd)
    udp.Send()
