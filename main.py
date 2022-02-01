from DroneClient import DroneClient
from DroneTypes import *
from vec2 import *
import time
from TangentBug import runTangentBug
import airsim.utils


def checkObstacles(client: DroneClient, pos: Vec2, goal: Vec2) -> bool:
    """
    checks if there is an obstacle in the path between the drone and the goal
    """
    plane_epsilon = 1
    colision_radius = 5

    # find the goal in body frame to match the cloud
    angle = client.getPose().orientation.z_rad
    point_cloud = client.getLidarData().points
    point_cloud_2d = [] if len(point_cloud) < 3 else [Vec2(point_cloud[i], point_cloud[i + 1])
                                                      for i in range(0, len(point_cloud), 3)
                                                      if abs(point_cloud[i + 2]) < plane_epsilon]
    # find the corresponding points in the world frame
    point_cloud_world = [p.rotate(angle) + pos for p in point_cloud_2d]

    # stop if apporching an obstacle
    return any(checkoverlapCircle(pos, goal, p, colision_radius) for p in point_cloud_world)


def startAndStop(client: DroneClient, goal: Position) -> bool:
    """
    flies the drone in the direction of the goal,
    stopping if there is an obstacle in the way.
    returns whether the goal was reached
    """
    drone_velocity = 10
    distance_epsilon = 3

    client.flyToPosition(goal.x_m, goal.y_m, goal.z_m, drone_velocity)
    while True:
        pos = client.getPose().pos
        goal_v = Vec2(goal.x_m, goal.y_m)
        pos_v = Vec2(pos.x_m, pos.y_m)

        if pos_v.distance(goal_v) <= distance_epsilon:
            client.flyToPosition(pos.x_m, pos.y_m, pos.z_m, 0.0001)
            return True
        elif checkObstacles(client, pos_v, goal_v):
            client.flyToPosition(pos.x_m, pos.y_m, pos.z_m, 0.0001)
            return False
        time.sleep(0.1)


if __name__ == "__main__":
    client = DroneClient()
    client.connect()


    print(client.isConnected())

    time.sleep(4)
    client.setAtPosition(-346, -700, -100)
    time.sleep(4)

    goal = Position()
    goal.x_m = -300
    goal.y_m = -400
    goal.z_m = -100
    startAndStop(client, goal)
