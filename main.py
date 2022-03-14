from DroneClient import DroneClient
from DroneTypes import *
from vec2 import *
import logging
import time
from TangentBug import TangentBug

logging.basicConfig(level=logging.DEBUG)


if __name__ == "__main__":
    client = DroneClient()
    client.connect()

    print(client.isConnected())

    # the plane on the z axis in which all the positions are found
    plane = -50
    # find the path from each position,
    # to the next one on the list
    positions = [
        Vec2(25, -570),
        Vec2(-1000, -1200),
        Vec2(-600, -300),
        Vec2(25, -570),
    ]

    time.sleep(2)
    client.setAtPosition(positions[0].x, positions[0].y, plane)
    time.sleep(1)

    bug = TangentBug(client, plane)
    for p in positions[1:]:
        bug.findPath(p)
