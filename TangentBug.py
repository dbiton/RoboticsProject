import math
import logging
import time
from typing import Generator, List, Optional, Set, Tuple, Dict, Iterable

from DroneClient import *
from DroneTypes import *
from vec2 import *


class TangentBug():
    colision_radius: float = 4
    """
    how far a line can be from a point in the plane,
    for it to count having colided with it
    """

    linking_distance: float = 6
    """
    how far two points need to be from each other,
    for the segment between them to be considered part of an obstacle
    """

    connection_distance: float = 8
    """
    how far two points need to be from each other to be considered part of the same obstacle
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

    response_time: float = 0.7
    """
    The time a single movement command should take for it to be registered as a valid command.
    must be greater than 0.6, but not too much.
    """

    memory_duration: float = 2
    """
    the time in seconds, that it takes for the drone to forget about a point,
    that it hasn't sensed since.
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

    boundary_distance: float = 5
    """
    the prefered distance the drone should be from the boundary while following it
    """

    offset_threshhold: float = 2
    """
    how far from the boundary while following it the drone should be,
    before course correction takes precedence over forward progress
    """

    high_velocity: float = 10
    """
    the speed at which the drone flies when there are no obstacles that need to be avoided
    """

    medium_velocity: float = 7.5
    """
    the speed at which the drone flies when obstacles are nearby,
    but they can be avoided with simple evasive maneuvers
    """

    low_velocity: float = 5
    """
    the speed at which the drone has to fly when there are obstacle nearby,
    that need to be approched, and avoided, carefully
    """

    stop_velocity: float = 0.00001
    """
    a velocity small enough for the current position to be reachable in time,
    as otherwise the drone might ignore the command.
    """

    client: DroneClient
    """
    the client with which the the algorithm communicates with the drone
    """

    plane: float
    """
    the z coordinate of the plane in which the algorithm is executed
    """

    raw_obstacle_points: Dict[Vec2, int] = {}
    obstacle_points: Dict[Vec2, int] = {}
    """
    the points detected by the drone on the way to the goal, in world frame,
    with the number of iterations since that point was last spotted

    (the raw points are the ones detected directly from the sensors,
    and the other ones, are modified according to the needs of the drone)
    """

    nearby_points: List[Vec2] = []
    """
    the obstacle points within the range of the drones sensor, in body frame
    """

    position: Vec2 = Vec2(0, 0)
    """
    the current position of the drone in world frame, based on the latest measurements
    """

    orientation: float = 0
    """
    the current orientation on the z plane of the drone in world frame, based on the latest measurements
    """

    goal: Vec2 = Vec2(0, 0)
    """
    the current goal which the drone is flying towards, in body frame
    """

    def __init__(self, client: DroneClient, plane: float) -> None:
        self.client = client
        self.plane = plane

    def stop(self):
        """
        make the drone hover in place
        """
        self.flyTo(Vec2(0, 0), self.stop_velocity)

    def flyTo(self, point: Vec2, velocity: float):
        """
        flies the drone to a given position in body frame
        """

        # calculate the distance the drone should travel in the given direction,
        # so that it takes atleast as much time
        distance = velocity * self.response_time
        normalized = point.normalize() * distance
        world_point = self.toWorldFrame(normalized)
        self.client.flyToPosition(
            world_point.x, world_point.y, self.plane, velocity)

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
            body_point = Vec2(point_cloud[i], point_cloud[i + 1])
            world_point = self.toWorldFrame(body_point)
            yield world_point

    def addObstaclePoint(self, point: Vec2):
        """
        add a point on an obstacle to the drones memory
        """
        self.raw_obstacle_points[point] = 0
        self.obstacle_points[point] = 0

        # add points in between points connected to this one,
        # to get smoother changes in the geometry
        addition = []
        for other in self.raw_obstacle_points:
            if point.distance(other) < self.linking_distance:
                addition.append((point + other) / 2)

        for p in addition:
            self.obstacle_points[p.round()] = 0

    def forgetOldPoints(self):
        """
        remove points previously detected by the drone,
        that haven't been seen for a while

        used to remove points that are either not on the current plane,
        or were produced by floating point imprecision,
        and avoid iterating over the entire map just to find the nearby points
        """

        forgotten = []
        for point, iterations in self.obstacle_points.items():
            if iterations > self.memory_duration / self.time_step:
                forgotten.append(point)
            else:
                # the point stays for another iteration
                self.obstacle_points[point] += 1
                if point in self.raw_obstacle_points:
                    self.raw_obstacle_points[point] += 1

        for p in forgotten:
            self.obstacle_points.pop(p, None)
            self.raw_obstacle_points.pop(p, None)

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

        self.forgetOldPoints()

        self.nearby_points = [self.toBodyFrame(p) for p in self.obstacle_points.keys()
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
        return p1.distance(p2) <= self.connection_distance

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
        # keep track of the last direction motion-to-goal when towards,
        # while it could still make progress, in world frame
        last_direction = self.goal.rotate(-self.orientation).normalize()

        while True:
            self.updateEnvironment()

            if self.goal.length() <= self.goal_epsilon:
                # arrived at the destination
                self.stop()
                return

            if following_boundary:
                if next(boundary_following_planner):
                    # motion to goal can make progress now,
                    # reset motion to goal and start it
                    motion_to_goal_planner = self.motionToGoal()
                    following_boundary = False

            else:
                point = next(motion_to_goal_planner)
                if point is None:
                    # motion to goal cant make progress,
                    # reset following the boundary and start it
                    boundary_following_planner = self.followBoundary(
                        last_direction)
                    following_boundary = True
                else:
                    last_direction = point.rotate(
                        -self.orientation).normalize()

            time.sleep(self.time_step)

    def motionToGoal(self) -> Generator[Optional[Vec2], None, None]:
        """
        flies the drone in the direction of the goal, if possible
        yields the next point it would have chosen to fly towards,
        in body frame, once no progress can be made with this path planner
        """
        last_heuristic_distance = math.inf
        while True:
            if self.checkObstaclesInPath():
                discontinuity_points = self.findDiscontinuityPoints()

                closest_point = min(discontinuity_points,
                                    key=lambda p: self.heuristicDistance(p))
                heuristic_distance = self.heuristicDistance(closest_point)

                if last_heuristic_distance < heuristic_distance:
                    yield

                else:
                    last_heuristic_distance = heuristic_distance
                    self.flyTo(closest_point, self.medium_velocity)
                    yield closest_point

            else:
                self.flyTo(self.goal, self.high_velocity)
                yield self.goal

    def getBlockingObstacle(self, path: Vec2) -> List[Vec2]:
        """
        finds all of the points on the obstacle blocking the path
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

        for point in reversed(clockwise_points):
            if any(self.checkPointsConnected(point, p) for p in obstacle):
                obstacle.append(point)
        return obstacle

    def findDiscontinuityPoints(self) -> Tuple[Vec2, Vec2]:
        """
        find the first and last points that are connected to the obstacle,
        in both the clockwise and counter-clockwise direction,
        """

        obstacle = self.getBlockingObstacle(self.goal)

        # rotate points away from the obstacle,
        # such that the new point is on the tangent to the colision circle,
        # to avoid coliding on the obstacle,
        # when no furthur discontinuity points can be found
        cw = min(obstacle, key=lambda p: self.goal.angle(p))
        cw_avoidance_angle = getFoVCoverage(cw, self.boundary_distance)
        cw = cw.rotate(-cw_avoidance_angle)

        ccw = max(obstacle, key=lambda p: self.goal.angle(p))
        ccw_avoidance_angle = getFoVCoverage(ccw, self.boundary_distance)
        ccw = ccw.rotate(ccw_avoidance_angle)

        return cw, ccw

    def heuristicDistance(self, point: Vec2) -> float:
        return point.length() + point.distance(self.goal)

    def followBoundary(self, path_hint: Optional[Vec2] = None) -> Generator[bool, None, None]:
        """
        follow the boundary of the obstacle currently blocking the path,
        yields True once the goal becomes is reachable, and False otherwise

        uses the path hint, if available, to choose a direction to follow,
        that matches the given direction vector in world frame
        """

        # keep track of the points on the obstacle being followed.
        #
        # since the position changes bewteen iterations,
        # keep them in world frame
        prev_followed_obstacle = list(self.toWorldFrame(p)
                                      for p in self.getBlockingObstacle(self.goal))

        followed_distance = math.inf

        # if no path hint is available, choose the direction based on the path to the goal
        if path_hint is None:
            path_hint = self.goal.rotate(-self.orientation)

        while True:

            reachable_distance = min((p.distance(self.goal) for p in self.getBlockingObstacle(
                self.goal)), default=max(self.goal.length() - self.sensor_range, 0))

            followed_obstacle = self.findConnectedPoints(self.toBodyFrame(p)
                                                         for p in prev_followed_obstacle)

            if len(followed_obstacle) == 0:
                # if the followed obstacle is unreachable,
                # try motion-to-goal again
                yield True

            followed_distance = min(
                followed_distance, min(p.distance(self.goal)
                                       for p in followed_obstacle))

            if followed_distance > reachable_distance:
                # end boundary following behavior, now that the goal is in reach
                yield True

            prev_followed_obstacle = list(
                self.toWorldFrame(p) for p in followed_obstacle)

            followed_point = min(followed_obstacle, key=lambda p: p.length())

            flight_direction, path_hint = self.getNextFollowDirection(
                followed_point, path_hint)

            self.flyTo(flight_direction, self.low_velocity)

            yield False

    def findConnectedPoints(self, points: Iterable[Vec2]) -> Set[Vec2]:
        # round the vectors to use avoid including the same point twice,
        # due to floating point precision loss
        connected = set(p.round() for p in points)
        nearby_points = set(p.round() for p in self.nearby_points)

        nearby_connected = connected.intersection(nearby_points)
        remaining_points = nearby_points.difference(nearby_connected)

        # establish connectivity between points,
        # by iterativly adding points that are directly connected,
        # untill all connected points are in the set
        while True:
            addition = set()

            for point in remaining_points:
                if any(self.checkPointsConnected(p, point) for p in nearby_connected):
                    addition.add(point)

            if len(addition) == 0:
                break
            else:
                nearby_connected.update(addition)
                remaining_points.difference_update(addition)

        return nearby_connected

    def getNextFollowDirection(self, followed_point: Vec2, path_hint: Vec2) -> Tuple[Vec2, Vec2]:
        """
        given an closest point on obstacle currently being followed,
        return the direction the drone should go to next to keep following it,
        and the direction of the path hint that should be used next time, in world frame
        """

        tangent = followed_point.perpendicular()

        # ensure that the direction taken by the drone is consistant across iterations
        tangent = tangent if abs(path_hint.rotate(
            self.orientation).angle(tangent)) <= math.pi / 2 else -tangent

        new_path_hint = tangent.rotate(-self.orientation).normalize()

        # maintain a fixed distance from the followed obstacle,
        # to both avoid hitting hit by being too close,
        # or hitting other obstacles by flying too far away
        distance_offset = followed_point.length() - self.boundary_distance

        course_correction = followed_point.normalize() * distance_offset

        # if the difference between the desired distance and the actual distance is too big,
        # ignore the tangent and focus on cource correcting,
        # to avoid taking wide turns or rotating around a point on the boundary
        flight_direction = tangent + course_correction\
            if abs(distance_offset) < self.offset_threshhold else course_correction

        return flight_direction, new_path_hint


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
