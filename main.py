from DroneClient import DroneClient
from DroneTypes import *
import time
from TangentBug import runTangentBug
import airsim.utils

if __name__ == "__main__":
    client = DroneClient()
    client.connect()


    print(client.isConnected())

    time.sleep(4)
    client.setAtPosition(-346, -700, -100)

    goal = Position()
    goal.x_m = -100
    goal.y_m = -100
    goal.z_m = -100
