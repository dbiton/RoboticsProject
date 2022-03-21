import math
import logging
import time
from typing import Generator, List, Optional, Set, Tuple, Dict, Iterable

from DroneClient import *
from DroneTypes import *
from quat import Quaternion
from vec2 import *


class TangentBug():
    colision_radius: float = 3
    """
    how far a line can be from a point in the plane,
    for it to count having colided with it
    """

    linking_distance: float = 6
    """
    how far two points need to be from each other,
    for the segment between them to be considered part of an obstacle
    """

    connection_distance: float = 9
    """
    how far two points need to be from each other to be considered part of the same obstacle
    """

    max_velocity: float = 10
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

    memory_duration: float = 5
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

    boundary_distance: float = 4
    """
    the prefered distance the drone should be from the boundary while following it
    """

    corridor_distance: float = 9
    """
    the prefered minimal distance between walls of a corridor,
    that the drone should attempt to travese,
    to ensure safe distance from both sides
    """

    offset_threshhold: float = 2
    """
    how far from the boundary while following it the drone should be,
    before course correction takes precedence over forward progress
    """

    vertigo_duration: float = 2
    """
    how long does the vertigo caused by sharp turns last, in seconds
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

    orientation3D: Quaternion = Quaternion(0, 0, 0, 1)
    """
    the full three dimentional orientation of the drone,
    used only for initial conversions to the 2D plane
    """

    goal: Vec2 = Vec2(0, 0)
    """
    the current goal which the drone is flying towards, in body frame
    """

    cur_corridor_width: float = math.inf
    """
    the actual width of the corridor the drone is inside of
    """

    vertigo: int = 0
    """
    the number of ticks untill the vertigo from sharp turns subsides,
    and the drone can move faster.
    """

    def __init__(self, client: DroneClient, plane: float) -> None:
        self.client = client
        self.plane = plane

    def stop(self):
        """
        make the drone hover in place
        """
        self.autoFlyTo(Vec2(0, 0))

    def autoFlyTo(self, point: Vec2, limit: float = max_velocity):
        """
        flies the drone to a given position in body frame,
        chooses the velocity automatically, based on the given point
        """
        length = point.length()
        if abs(Vec2(1, 0).angle(point)) > math.pi / 6:
            # flying through a sharp turn, expeciencing vertigo as a result
            self.vertigo = round(2 / self.time_step)

        velocity: float
        if length < 0.0001:
            # stop at current position
            velocity = self.stop_velocity
        else:
            # take atleast a second to respond to obstacles ahead
            safety_velocity = min((p.length()
                                   for p in self.nearby_points), default=math.inf)
            if self.vertigo <= 0:
                safety_velocity *= 2
            else:
                self.vertigo -= 1

            velocity = min(limit, safety_velocity)

        # calculate the distance the drone should travel in the given direction,
        # so that it takes atleast as much time
        distance = velocity * self.response_time
        world_point = self.toWorldFrame(point.normalize() * distance)
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
            point = Quaternion(
                point_cloud[i], point_cloud[i + 1], point_cloud[i + 2], 0)
            rotated = self.orientation3D * point * self.orientation3D.conjugate()
            world_point = Vec2(rotated.x, rotated.y) + self.position
            yield world_point

    def addObstaclePoint(self, point: Vec2):
        """
        add a point on an obstacle to the drones memory
        """
        self.obstacle_points[point.round()] = 0

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
        self.orientation3D = Quaternion.from_euler_angles(pose.orientation.x_rad,
                                                          pose.orientation.y_rad,
                                                          pose.orientation.z_rad)
        position = Vec2(pose.pos.x_m, pose.pos.y_m)

        world_goal = self.toWorldFrame(self.goal)
        self.position = position
        self.orientation = pose.orientation.z_rad
        self.goal = self.toBodyFrame(world_goal)
        self.cur_corridor_width = self.findCorridorWidth()

        for point in self.detectObstacles():
            self.addObstaclePoint(point)

        self.forgetOldPoints()

        # ignore points that are too close to the drone,
        # which might make it seem like the drone is inside the wall
        self.nearby_points = [self.toBodyFrame(p) for p in self.obstacle_points.keys()
                              if 1 < p.distance(self.position) < self.sensor_range]

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
                point = next(boundary_following_planner, None)
                if point is None:
                    # motion to goal can make progress now,
                    # reset motion to goal and start it
                    motion_to_goal_planner = self.motionToGoal()
                    following_boundary = False
                else:
                    self.autoFlyTo(point)

            else:
                point = next(motion_to_goal_planner, None)
                if point is None:
                    # motion to goal cant make progress,
                    # reset following the boundary and start it
                    boundary_following_planner = self.followBoundary(
                        last_direction)
                    following_boundary = True
                else:
                    self.autoFlyTo(point)
                    last_direction = point.rotate(
                        -self.orientation).normalize()

            time.sleep(self.time_step)

    def findSegmentColision(self, path: Vec2) -> Optional[Vec2]:
        """
        returns the first point on an obstacle which intersects with the given path from the origin,
        if the segment intersects with an obstacle
        """
        return min((p for p in self.nearby_points if checkoverlapCircle(
                    Vec2(0, 0), path, p, self.colision_radius)),
                   key=lambda p: p.length(), default=None)

    def motionToGoal(self) -> Generator[Vec2, None, None]:
        """
        follows the most direct path to the goal, if possible
        yields the next point on that path, in body frame,
        while this planner can make progress,
        """
        last_heuristic_distance = math.inf
        while True:
            if self.checkObstaclesInPath():
                discontinuity_points = self.findDiscontinuityPoints()

                closest_point = min(discontinuity_points,
                                    key=lambda p: self.heuristicDistance(p))
                heuristic_distance = self.heuristicDistance(closest_point)

                if last_heuristic_distance < heuristic_distance:
                    return

                else:
                    last_heuristic_distance = heuristic_distance
                    yield closest_point

            else:
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

    def getFollowedBoundary(self, followed_point: Vec2) -> Generator[Vec2, None, None]:
        """
        returns the points on the boundary near the currently followed point,
        that should be considered as the part of the obstacle being followed.
        """
        corridor_ratio = self.cur_corridor_width / self.corridor_distance
        # ensure that the resized circle around the followed point,
        # does not include the other side of the corridor
        resize = min(1, 0.9 * corridor_ratio)

        for point in self.nearby_points:
            if point.distance(followed_point) < resize * self.corridor_distance:
                yield point

    def followBoundary(self, prev_path_hint: Optional[Vec2] = None) -> Generator[Vec2, None, None]:
        """
        follow the boundary of the obstacle currently blocking the path,
        yields the next point on the path untill the goal becomes reachable.

        uses the path hint, if available, to choose a direction to follow,
        that matches the given direction vector in world frame
        """

        min_followed_distance = math.inf

        right_follow = None

        # keep track of all the nearby points that were previously followed,
        # to ensure the drone doesn't stick to disconnected obstacles
        prev_followed_obstacle = [self.toWorldFrame(p)
                                  for p in self.getBlockingObstacle(self.goal)]

        while True:

            # ensure that the obstacle contains only points that are currently nearby
            followed_obstacle = set(self.toBodyFrame(p).round()
                                    for p in prev_followed_obstacle)
            followed_obstacle.intersection_update(p.round()
                                                  for p in self.nearby_points)

            followed_point = min(followed_obstacle,
                                 key=lambda p: p.length(), default=None)

            if followed_point is None:
                # if the followed obstacle is unreachable, try motion to goal again
                return

            # add points near the new followed point to follow along in that direction
            followed_obstacle.update(
                self.getFollowedBoundary(followed_point))

            # ensure that obstacles in the way to the followed obstalce are not ignored
            followed_obstacle.update(
                p for p in self.nearby_points if p.length() < self.boundary_distance)

            prev_followed_obstacle = [
                self.toWorldFrame(p) for p in followed_obstacle]

            if right_follow is None:
                # helps convince pyright linter that followed point is not None in this branch
                fp = followed_point

                # if no path hint is available, choose the direction based on the path to the goal
                path_hint = prev_path_hint.rotate(
                    self.orientation) if prev_path_hint is not None else self.goal

                # find the direction to follow that is closest to the path the drone is already going towards
                right_follow = min(
                    [True, False], key=lambda b: abs(self.getNextFollowPoint(fp, b).angle(path_hint)))

            # consider only points near the followed point to be part of the followed obstacle,
            # to avoid staying in boundary following mode due to unreachable points.
            # the points should be on the side of the corridor being followed.
            followed_distance = min(p.distance(
                self.goal) for p in self.getFollowedBoundary(followed_point))

            min_followed_distance = min(
                followed_distance, min_followed_distance)

            # the goal is reachable if there is any point in free space,
            # which is closer to the goal than the followed obstacle.
            # if the goal is blocked, the boundary around the first point blocking it,
            # is at the edge of free space
            blocking_point = self.findSegmentColision(self.goal)
            reachable_distance = max(self.goal.length() - self.sensor_range, 0)\
                if blocking_point is None else min(p.distance(self.goal)
                                                   for p in self.getFollowedBoundary(blocking_point))

            if min_followed_distance > reachable_distance:
                # end boundary following behavior, now that the goal is in reach
                return

            flight_direction = self.getNextFollowPoint(
                followed_point, right_follow)

            yield flight_direction

    def findCorridorWidth(self) -> float:
        """
        finds the width of the corridor the drone is in,
        if the drone is not in a corridor, that distance is infinity.
        """
        closest_point = min(
            self.nearby_points, key=lambda p: p.length(), default=None)

        if closest_point is None:
            return math.inf

        opposing_distance = min((p.length() for p in self.nearby_points
                                if abs(closest_point.angle(p)) > math.pi / 2), default=math.inf)
        return closest_point.length() + opposing_distance

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
        and the direction of the path hint that should be used next time, in body frame
        """

        tangent = followed_point.perpendicular()

        # ensure that the direction taken by the drone is consistant across iterations
        tangent = tangent if abs(path_hint.angle(
            tangent)) <= math.pi / 2 else -tangent

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

        return flight_direction, tangent

    def getNextFollowPoint(self, followed_point: Vec2, right_follow: bool) -> Vec2:
        """
        given a point on an obstacle being followed,
        and whether the obstacle is being followed from the left or from the right,
        return the point the drone should go to next to keep following it, in body frame
        """

        angle_sign = 1 if right_follow else -1

        away_point = Vec2(0, 0)
        max_angle = -math.inf

        for point in self.getFollowedBoundary(followed_point):
            # ensure that the distance from the boundary is small enough,
            # to avoid being closer to the other side of the corridor
            resize = min(1, 0.4 * self.cur_corridor_width)
            radius = min(resize * self.boundary_distance,
                         0.9999 * point.length())

            # rotate away from the obstacle to avoid coliding with it
            avoidance_angle = getFoVCoverage(point, radius)
            rotated = point.rotate(avoidance_angle * angle_sign)

            # find the point that would avoid all other points on the obstacle as well
            angle = angle_sign * followed_point.angle(rotated)
            if max_angle < angle:
                max_angle = angle
                away_point = rotated

        return away_point


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
