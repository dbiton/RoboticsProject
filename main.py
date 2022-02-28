from DroneClient import DroneClient
from DroneTypes import *
from vec2 import *
import logging
import time
from TangentBug import SimpleBug

logging.basicConfig(level=logging.DEBUG)


if __name__ == "__main__":
    client = DroneClient()
    client.connect()

    print(client.isConnected())

    time.sleep(4)
    client.setAtPosition(-200, -800, -100)
    time.sleep(4)

    goal = Vec2(-600, -800)
    plane = -100

    bug = SimpleBug(client, plane)
    bug.findPath(goal)
