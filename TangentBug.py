from operator import itemgetter
import math
import time
from typing import Generator, Set, Tuple

from DroneClient import *
from DroneTypes import *
from vec2 import *


def vectorNormalized(p: Tuple[float, float]):
    return vectorMul(p, 1.0 / vectorLen(p))


def vectorMul(p: Tuple[float, float], a: float):
    return p[0] * a, p[1] * a


def vectorAdd(p0: Tuple[float, float], p1: Tuple[float, float]):
    return p0[0] + p1[0], p0[1] + p1[1]


def vectorSub(p0: Tuple[float, float], p1: Tuple[float, float]):
    return p0[0] - p1[0], p0[1] - p1[1]


def cartesianToPolar(p: Tuple[float, float]):
    return math.sqrt(p[0] ** 2 + p[1] ** 2), math.atan2(p[1], p[0])


def polarToCartesian(p: Tuple[float, float]):
    return p[0] * math.cos(p[1]), p[0] * math.sin(p[1])


def distance(p0: Tuple[float, float], p1: Tuple[float, float]):
    return math.sqrt((p0[0] - p1[0]) ** 2 + (p0[1] - p1[1]) ** 2)


def vectorLen(p: Tuple[float, float]):
    return math.sqrt((p[0]) ** 2 + (p[1]) ** 2)


def vector(src: Tuple[float, float], dst: Tuple[float, float]):
    return dst[0] - src[0], dst[1] - dst[1]


def runTangentBug(client: DroneClient, goal: Position):
    tangent_bug = TangentBug()
    tangent_bug.setGoal((goal.x_m, goal.y_m))

    drone_vel = 10

    plane_epsilon = 1
    while True:
        point_cloud = client.getLidarData().points
        drone_pos = client.getPose().pos
        if len(point_cloud) < 3:
            point_cloud_2d = []
        else:
            point_cloud_2d = [(point_cloud[i], point_cloud[i + 1]) for i in range(0, len(point_cloud), 3)
                              if abs(point_cloud[i + 2]) < plane_epsilon]
            print(point_cloud_2d)
        drone_pos_2d = (drone_pos.x_m, drone_pos.y_m)
        dst_2d = tangent_bug.pathfind(drone_pos_2d, point_cloud_2d)
        client.flyToPosition(dst_2d[0], dst_2d[1], drone_pos.z_m, drone_vel)
        time.sleep(0.1)


class TangentBug:
    def __init__(self):
        self.grid_size = 1
        self.memory = list()
        self.prev_heuristic_distance = float("inf")
        self.is_following_boundary = False
        self.goal = (0, 0)
        self.segment_rot_epsilon = 0.01
        self.segment_len_epsilon = 1
        self.goal_distance_epsilon = 1
        self.building_distance_epsilon = 15

    def setGoal(self, goal: Tuple[float, float]):
        self.goal = goal

    def calcEdgePoints(self, polar_points: list):
        edges = []
        for i in range(len(polar_points)):
            p0 = polar_points[i - 1]
            p1 = polar_points[i]
            if abs(p0[0] - p1[0]) > self.segment_len_epsilon or abs(p0[1] - p1[1]) > self.segment_rot_epsilon:
                edges.append(polarToCartesian(p0))
        return edges

    def goalPathIntersectsSegment(self, pos: Tuple[float, float], polar_points: list):
        angle_goal = math.atan2(self.goal[1] - pos[1], self.goal[0] - pos[0])
        path_distances = [p[0] * abs(math.sin(p[1] - angle_goal))
                          for p in polar_points]
        return min(path_distances, default=float('inf')) < self.building_distance_epsilon

    def motionToGoal(self, pos):
        polar_points = [cartesianToPolar(vectorSub(p, pos))
                        for p in self.memory]
        polar_points.sort(key=itemgetter(1))
        if self.goalPathIntersectsSegment(pos, polar_points):
            edge_points = self.calcEdgePoints(polar_points)
            heuristic_distances = [
                distance(pos, o) + distance(o, self.goal) for o in edge_points]
            min_heuristic_distance = min(
                heuristic_distances, default=float('inf'))
            if min_heuristic_distance < self.prev_heuristic_distance:
                self.prev_heuristic_distance = min_heuristic_distance
                i = heuristic_distances.index(min_heuristic_distance)
                edge_point = edge_points[i]
                return vector(pos, edge_point)
            else:
                self.prev_heuristic_distance = float("inf")
                self.is_following_boundary = True
                return self.followBoundary(pos)
        else:
            return vector(pos, self.goal)

    def followBoundary(self, pos):
        point_distances = [distance(pos, p) for p in self.memory]
        i = point_distances.index(min(point_distances))
        print('point distances', point_distances)
        boundary_point = self.memory[i]
        print('boundary point', point_distances)
        reach_distance = distance(pos, self.goal)
        followed_distance = distance(boundary_point, self.goal)
        print('reach', reach_distance, 'followed', followed_distance)
        if reach_distance >= followed_distance:
            print("b0")
            boundary_normal = vector(boundary_point, pos)
            boundary_tangent = vectorNormalized(
                (boundary_normal[1], -boundary_normal[0]))
            print('dir', boundary_tangent)
            return vectorAdd(pos, vectorMul(boundary_tangent, 10))
        else:
            print("b1")
            self.is_following_boundary = False
            return self.motionToGoal(pos)

    def add_to_memory(self, pos: Tuple[float, float], points: list):
        grid_points = [(math.floor((pos[0] + p[0]) / self.grid_size) * self.grid_size,
                        math.floor((pos[1] + p[1]) / self.grid_size) * self.grid_size) for p in points]
        grid_points = [p for p in grid_points if p not in self.memory]
        self.memory += grid_points

    def pathfind(self, pos: Tuple[float, float], points: list):
        print('following boundary: ', self.is_following_boundary)
        if distance(pos, self.goal) < self.goal_distance_epsilon:
            return pos
        self.add_to_memory(pos, points)
        if self.is_following_boundary:
            return self.followBoundary(pos)
        else:
            return self.motionToGoal(pos)


# smaller steps towards tangent-bug

def detectObstacles(client: DroneClient, pos: Vec2) -> Generator[Vec2, None, None]:
    """
    find points around the drone in the world frame, detected by the drones LIDAR
    """
    plane_epsilon = 1
    angle = client.getPose().orientation.z_rad
    point_cloud = client.getLidarData().points

    if len(point_cloud) < 3:
        # the cloud is empty, no points where observed
        return

    for i in range(0, len(point_cloud), 3):

        if abs(point_cloud[i + 2]) >= plane_epsilon:
            # ignore points outside the flight plane
            continue

        body_point = Vec2(point_cloud[i], point_cloud[i + 1])
        world_point = body_point.rotate(angle) + pos
        yield world_point


def checkObstaclesInPath(pos: Vec2, goal: Vec2, obstacle_points: Set[Vec2]) -> bool:
    """
    checks if there is an obstacle in the path between the drone and the goal
    """
    colision_radius = 5

    # stop if apporching an obstacle
    return any(checkoverlapCircle(pos, goal, p, colision_radius) for p in obstacle_points)


def startAndStop(client: DroneClient, goal: Position) -> bool:
    """
    flies the drone in the direction of the goal,
    stopping if there is an obstacle in the way.
    returns whether the goal was reached
    """
    drone_velocity = 10
    distance_epsilon = 3

    obstacle_points: Set[Vec2] = set()

    client.flyToPosition(goal.x_m, goal.y_m, goal.z_m, drone_velocity)
    while True:
        pos = client.getPose().pos
        goal_v = Vec2(goal.x_m, goal.y_m)
        pos_v = Vec2(pos.x_m, pos.y_m)

        for p in detectObstacles(client, pos_v):
            obstacle_points.add(p)

        if pos_v.distance(goal_v) <= distance_epsilon:
            client.flyToPosition(pos.x_m, pos.y_m, pos.z_m, 0.0001)
            return True
        elif checkObstaclesInPath(pos_v, goal_v, obstacle_points):
            client.flyToPosition(pos.x_m, pos.y_m, pos.z_m, 0.0001)
            return False
        time.sleep(0.1)


# used in the bonux task for keeping track of points in the entire map
class ObstacleMap:
    """
    a map describing the obstacles observed by the drone in its path
    """

    # each element corresponds to a meter by meter pixel on the map
    # the indicies are mapped to coordinates,
    # in the range specified by *_start and *_end for each axis
    #
    # an inhabited pixel is marked with 1 and an empty one with 0
    array: bytearray
    y_start: int
    y_end: int
    x_start: int
    x_end: int

    def __init__(self, y_start: int = -1300, y_end: int = 200, x_start: int = -1300, x_end: int = 200) -> None:
        self.y_start = y_start
        self.y_end = y_end
        self.x_start = x_start
        self.x_end = x_end
        self.array = bytearray((y_end - y_start) * (x_end - x_start))

    def row_size(self) -> int:
        return (self.y_end - self.y_start)

    def column_size(self) -> int:
        return (self.x_end - self.x_start)

    def mark(self, point: Vec2):
        """
        mark the given point as an obstacle on the map
        """
        x = math.floor(point.x)
        y = math.floor(point.y)

        i = y - self.y_start
        j = x - self.x_start

        self.array[i * self.row_size() + j] = 1
