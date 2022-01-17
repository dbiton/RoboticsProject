from DroneClient import DroneClient
from DroneTypes import *
import time
from Pathfinder import Pathfinder
import airsim.utils

if __name__ == "__main__":
    client = DroneClient()
    client.connect()

    print(client.isConnected())

    time.sleep(4)
    client.setAtPosition(-346, -700, -100)

    # time.sleep(3)
    # client.flyToPosition(-346, -420, -100, 10)

    goal = Position()
    goal.x_m = -100
    goal.y_m = -100
    goal.z_m = -100

    path_finder = Pathfinder(client, goal)

    # path_finder.FindPath()
    while True:
        print(client.getLidarData())
        time.sleep(1)
