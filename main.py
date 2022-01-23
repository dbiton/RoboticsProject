from DroneClient import DroneClient
from DroneTypes import *
import time
from TangentBug import TangentBug
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

    tangent_bug = TangentBug()

    drone_vel = 10

    plane_epsilon = 1
    while True:
        point_cloud = client.getLidarData().points
        drone_pos = client.getPose().pos
        point_cloud_2d = [(p.x_m, p.y_m) for p in point_cloud if abs(p.z_m-drone_pos.z_m) < plane_epsilon]
        drone_pos_2d = (drone_pos.x_m, drone_pos.y_m)
        dst_2d = tangent_bug.pathfind(drone_pos_2d, point_cloud_2d)
        client.flyToPosition(dst_2d[0], dst_2d[1], drone_pos.z_m, drone_vel)
