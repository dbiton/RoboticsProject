from DroneClient import DroneClient
from DroneTypes import *
from vec2 import *
import time
from TangentBug import runTangentBug, startAndStop


if __name__ == "__main__":
    client = DroneClient()
    client.connect()

    print(client.isConnected())

    time.sleep(4)
    client.setAtPosition(-346, -700, -100)
    time.sleep(4)

    goal = Position()
    goal.x_m = -400
    goal.y_m = -300
    goal.z_m = -100
    startAndStop(client, goal)
