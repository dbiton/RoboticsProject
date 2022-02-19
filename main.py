from DroneClient import DroneClient
from DroneTypes import *
from vec2 import *
import logging
import time
from TangentBug import SimpleBug, runTangentBug

logging.basicConfig(level=logging.DEBUG)


if __name__ == "__main__":
    client = DroneClient()
    client.connect()

    print(client.isConnected())

    time.sleep(4)
    client.setAtPosition(-346, -700, -100)
    time.sleep(4)

    goal = Vec2(-400, -300)
    plane = -100

    bug = SimpleBug(client, plane)
    bug.findPath(goal)
