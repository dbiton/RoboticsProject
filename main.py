from DroneClient import DroneClient
from DroneTypes import *
import time
import airsim.utils

def fly(drone: DroneClient, target: Position):
    while True:
        print(drone.getLidarData())
        print(drone.getPose())
        time.sleep(1)

if __name__ == "__main__":
    client = DroneClient()
    client.connect()

    print(client.isConnected())

    time.sleep(4)
    client.setAtPosition(-346, -700, -100)

    # time.sleep(3)
    # client.flyToPosition(-346, -420, -100, 10)

    target_pos = Position()
    target_pos.x_m = -250
    target_pos.y_m = -300
    target_pos.z_m = -100

    fly(client, target_pos)
