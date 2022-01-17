from DroneClient import DroneClient
from DroneTypes import *
import time
import airsim.utils

def fly(start: Position, end: Position):
    while True:
        print(client.getLidarData())
        time.sleep(1)

if __name__ == "__main__":
    client = DroneClient()
    client.connect()

    print(client.isConnected())

    time.sleep(4)
    client.setAtPosition(-346, -700, -100)

    # time.sleep(3)
    # client.flyToPosition(-346, -420, -100, 10)

    start_pos = Position()
    start_pos.x_m = -346
    start_pos.y_m = -700
    start_pos.z_m = -100

    fly(start_pos, start_pos)
