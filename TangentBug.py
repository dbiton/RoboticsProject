from operator import itemgetter
import math
import logging
import time
from typing import Generator, List, Optional, Set, Tuple

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

class SimpleBug():
    plane_epsilon: float = 1
    """
    a far a point can be away from the drone in the z axis,
    # for it to count as being on the same plane
    """

    colision_radius: float = 3
    """
    how far a line can be from a point in the plane,
    for it to count having colided with it
    """

    drone_velocity: float = 10
    """
    the maximum velocity the drone can have while flying.
    can be lower (for example while turing)
    """

    time_step: float = 1 / 50
    """
    the interval, in seconds, between each iteration of the algorithm,
    to ensure the busy loop isn't doing redundant computation
    """

    goal_epsilon: float = 3
    """
    how far the current position of the drone can be from the goal,
    for it to count as having reached the goal
    """

    sensor_range: float = 35
    """
    the maximum distance away from the drone, a point can be detected by its sensors,
    while a point outside that range, it is ignored by the path finding algorithm.
    """

    boundary_distance: float = 6
    """
    the prefered distance the drone should be from the boundary while following it
    """

    client: DroneClient
    """
    the client with which the the algorithm communicates with the drone
    """

    obstacle_points: Set[Vec2]
    """
    the points detected by the drone on the way to the goal, in world frame
    """

    nearby_points: List[Vec2]
    """
    the obstacle points within the range of the drones sensor, in body frame
    """

    position: Vec2
    """
    the current position of the drone in world frame, based on the latest measurements
    """

    orientation: float
    """
    the current orientation on the z plane of the drone in world frame, based on the latest measurements
    """

    goal: Vec2
    """
    the current goal which the drone is flying towards, in body frame
    """

    plane: float
    """
    the z coordinate of the plane in which the algorithm is executed
    """

    def __init__(self, client: DroneClient, plane: float) -> None:
        self.client = client
        self.plane = plane
        self.obstacle_points = set()
        self.position = Vec2(0, 0)
        self.orientation = 0.0
        self.goal = Vec2(0, 0)

    def stop(self):
        """
        make the drone hover in place
        """
        self.flyTo(Vec2(0, 0), velocity=0.0001)

    def flyTo(self, point: Vec2, velocity: Optional[float] = None):
        """
        flies the drone to a given position in body frame
        """
        drone_velocity = self.drone_velocity if velocity is None else velocity
        world_point = self.toWorldFrame(point)
        self.client.flyToPosition(
            world_point.x, world_point.y, self.plane, drone_velocity)

    def toBodyFrame(self, point: Vec2) -> Vec2:
        """
        given a point in world frame,
        convert it to the equivalent point in the drones body frame
        """
        return (point - self.position).rotate(-self.orientation)

    def toWorldFrame(self, point: Vec2) -> Vec2:
        """
        given a point in drones body frame,
        convert it to the equivalent point in the world frame
        """
        return point.rotate(self.orientation) + self.position

    def setGoal(self, goal: Vec2):
        """
        sets a new goal point for the drone,
        given its position in the world frame
        """
        self.goal = self.toBodyFrame(goal)

    def detectObstacles(self) -> Generator[Vec2, None, None]:
        """
        find points around the drone, detected by the drones LIDAR,
        yielded in world frame
        """
        point_cloud = self.client.getLidarData().points

        if len(point_cloud) < 3:
            # the cloud is empty, no points where observed
            return

        for i in range(0, len(point_cloud), 3):
            plane_delta = point_cloud[i + 2]
            if abs(plane_delta) >= self.plane_epsilon:
                # ignore points outside the flight plane
                continue
            body_point = Vec2(point_cloud[i], point_cloud[i + 1])
            world_point = self.toWorldFrame(body_point)
            yield world_point

    def addObstaclePoint(self, point: Vec2):
        """
        add a point on an obstacle to the drones memory
        """
        # round up the coordinates of the point
        # to avoid storing redundant points
        x = round(point.x)
        y = round(point.y)
        self.obstacle_points.add(Vec2(x, y))

    def updateEnvironment(self):
        """
        update the state of the drone and surrounding obstacles,
        based on the latest data from the sensors
        """
        pose = self.client.getPose()
        position = Vec2(pose.pos.x_m, pose.pos.y_m)

        world_goal = self.toWorldFrame(self.goal)
        self.position = position
        self.orientation = pose.orientation.z_rad
        self.goal = self.toBodyFrame(world_goal)

        for point in self.detectObstacles():
            self.addObstaclePoint(point)

        self.nearby_points = [self.toBodyFrame(p) for p in self.obstacle_points
                              if p.distance(self.position) < self.sensor_range]

    def checkObstaclesInPath(self) -> bool:
        """
        checks if there is an obstacle in the path between the drone and the goal
        """
        return any(checkoverlapCircle(Vec2(0, 0), self.goal, p, self.colision_radius) for p in self.nearby_points)

    def checkPointsConnected(self, p1: Vec2, p2: Vec2) -> bool:
        """
        returns whether the colision circles of the two given points intersect,
        indicating that they are conneced.
        """
        return p1.distance(p2) <= 2 * self.colision_radius

    def findPath(self, goal: Vec2):
        """
        flies the drone towards the goal,
        avoiding obstacles as necessary using the tangent bug algorithm
        """
        self.setGoal(goal)
        self.updateEnvironment()

        following_boundary = False

        boundary_following_planner = self.followBoundary()
        motion_to_goal_planner = self.motionToGoal()

        while True:
            self.updateEnvironment()

            if self.goal.length() <= self.goal_epsilon:
                # arrived at the destination
                self.stop()
                return

            if self.client.checkColision():
                self.client.reset()
                return

            if following_boundary:
                if next(boundary_following_planner):
                    # motion to goal can make progress now,
                    # reset boundary following and start the motion to goal
                    boundary_following_planner = self.followBoundary()
                    following_boundary = False

            else:
                if not next(motion_to_goal_planner):
                    # motion to goal cant make progress,
                    # reset iy and start following the boundary
                    motion_to_goal_planner = self.motionToGoal()
                    following_boundary = True

            time.sleep(self.time_step)

    def motionToGoal(self) -> Generator[bool, None, None]:
        """
        flies the drone in the direction of the goal, if possible
        yields True so long as progress can be made with this path planner
        """
        last_heuristic_distance = math.inf
        while True:
            if self.checkObstaclesInPath():
                discontinuity_points = self.findDiscontinuityPoints()

                closest_point = min(discontinuity_points,
                                    key=lambda p: self.heuristicDistance(p))
                heuristic_distance = self.heuristicDistance(closest_point)

                if last_heuristic_distance < heuristic_distance:
                    yield False

                else:
                    last_heuristic_distance = heuristic_distance
                    self.flyTo(closest_point)

            else:
                self.flyTo(self.goal)

            yield True

    def getBlockingObstacle(self, path: Vec2) -> List[Vec2]:
        """
        finds all of the points on the obstacle blocking the path,
        sorted by their angle relative to said path
        """

        # the points of the blocking obstacle, connected by their colision circles
        obstacle = []

        counter_clockwise_points = []
        clockwise_points = []

        self.nearby_points.sort(key=lambda p: path.angle(p))

        # since a point clockwise to the goal can be connected to a point counter clockwise,
        # all points directly on the path have to be found before deciding whether the rest are connected
        for point in self.nearby_points:
            if checkoverlapCircle(Vec2(0, 0), path, point, self.colision_radius):
                obstacle.append(point)
            elif path.angle(point) > 0:
                counter_clockwise_points.append(point)
            else:
                clockwise_points.append(point)

        # find points connected to the obstacle from either end, while maintaining the order,
        # so that the first and last points in the obstacle are the discontinuity points
        for point in counter_clockwise_points:
            if any(self.checkPointsConnected(point, p) for p in obstacle):
                obstacle.append(point)

        # reverse the obstacle so that points would be appended in reverse order
        obstacle.reverse()
        for point in reversed(clockwise_points):
            if any(self.checkPointsConnected(point, p) for p in obstacle):
                obstacle.append(point)
        return obstacle

    def findDiscontinuityPoints(self) -> Tuple[Vec2, Vec2]:
        """
        find the first and last points that are connected to the obstacle,
        in both the clockwise or counter-clockwise direction,
        """

        obstacle = self.getBlockingObstacle(self.goal)
        return obstacle[0], obstacle[-1]

    def heuristicDistance(self, point: Vec2) -> float:
        return point.length() + point.distance(self.goal)

    def followBoundary(self) -> Generator[bool, None, None]:
        """
        follow the boundary of the obstacle currently blocking the path,
        yields True once the goal becomes is reachable, and False otherwise
        """

        # keep track of the point followed in the previous iteration,
        # to find the obstacle being followed.
        #
        # since the position changes bewteen iterations,
        # keep it in world frame
        prev_followed_point = self.toWorldFrame(self.goal)

        followed_distance = math.inf

        while True:

            # for simplicity's sake, the reachable distance is computed,
            # as though no point outside the drone is visable.
            #
            # this causes the drone to stop following the boundary later than is possible,
            # but is otherwise still correct (for a non-zero width obstacle)
            reachable_distance = self.goal.length()

            old_obstacle_direction = self.toBodyFrame(prev_followed_point)

            followed_obstacle = self.getBlockingObstacle(
                old_obstacle_direction)

            if len(followed_obstacle) == 0:
                # if the followed obstacle is unreachable,
                # try motion-to-goal again
                yield True

            followed_point = min(followed_obstacle, key=lambda p: p.length())

            followed_distance = min(
                followed_distance, followed_point.distance(self.goal))

            if followed_distance > reachable_distance:
                # end boundary following behavior, now that the goal is in reach
                yield True

            tangent = followed_point.perpendicular()

            # ensure that the direction taken by the drone is consistant across iterations
            tangent = tangent if abs(Vec2(1, 0).angle(
                tangent) <= math.pi / 2) else -tangent

            # maintain a fixed distance from the followed obstacle,
            # to both avoid hitting hit by being too close,
            # or hitting other obstacles by flying too far away
            distance_offset = followed_point.length() - self.boundary_distance

            course_correction = followed_point * \
                (distance_offset / followed_point.length())

            flight_direction = tangent + course_correction

            self.flyTo(flight_direction, velocity=self.drone_velocity / 2)

            prev_followed_point = self.toWorldFrame(followed_point)

            yield False


class ObstacleMap:  # used in the bonux task for keeping track of points in the entire map
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
