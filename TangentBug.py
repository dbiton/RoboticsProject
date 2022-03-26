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

    max_ubran_velocity: float = 10
    """
    the maximum velocity the drone can have while flying near buildings.
    """

    max_highway_velocity: float = 25
    """
    the maximum velocity the drone can have while flying above the open road.
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

    known_waypoint: Vec2 = Vec2(-1178, -509)
    """
    a point on the map, known for being an a road intersection,
    used as a reference for finding other intersections on the grid
    """

    grid_x_interval: float = 170.7
    grid_y_interval: float = 194.7
    """
    the values along the x and y coordinates, such that after each step in either the x or y direction,
    there is another intersection on the grid (within the borders of the map).
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

    def autoFlyTo(self, point: Vec2, limit: float = max_ubran_velocity):
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
                safety_velocity *= 1.5
            else:
                self.vertigo -= 1

            # slow down next to goal, to avoid hitting obstacles near waypoints
            velocity = min(limit, safety_velocity, self.goal.length() / 2)

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

    def findPath(self, goal: Vec2, limit: float = max_ubran_velocity):
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
                # if the drone ended up following a boundary,
                # it might be off the road, dont speed up
                limit = self.max_ubran_velocity
                point = next(boundary_following_planner, None)
                if point is None:
                    # motion to goal can make progress now,
                    # reset motion to goal and start it
                    motion_to_goal_planner = self.motionToGoal()
                    following_boundary = False
                else:
                    self.autoFlyTo(point, limit=limit)

            else:
                point = next(motion_to_goal_planner, None)
                if point is None:
                    # motion to goal cant make progress,
                    # reset following the boundary and start it
                    boundary_following_planner = self.followBoundary(
                        last_direction)
                    following_boundary = True
                else:
                    self.autoFlyTo(point, limit=limit)
                    last_direction = point.rotate(
                        -self.orientation).normalize()

            time.sleep(self.time_step)

    def findTaxicabPath(self, goal: Vec2):
        position = self.client.getPose().pos
        pos = Vec2(position.x_m, position.y_m)

        # find the path through waypoints first
        # then do the rest with normal tangent bug
        rel_pos = pos - self.known_waypoint
        rel_goal = goal - self.known_waypoint
        start_waypoint = Vec2(round(rel_pos.x / self.grid_x_interval) * self.grid_x_interval,
                              round(rel_pos.y / self.grid_y_interval) * self.grid_y_interval) + self.known_waypoint
        end_waypoint = Vec2(round(rel_goal.x / self.grid_x_interval) * self.grid_x_interval,
                            round(rel_goal.y / self.grid_y_interval) * self.grid_y_interval) + self.known_waypoint

        # seperate the path between the waypoints to a horizontal and vertical part,
        # to match the roads between them. in those roads the drone can be faster.
        self.findPath(start_waypoint, limit=self.max_ubran_velocity)
        self.findPath(Vec2(end_waypoint.x, start_waypoint.y),
                      limit=self.max_highway_velocity)
        self.findPath(end_waypoint, limit=self.max_highway_velocity)
        self.findPath(goal, limit=self.max_ubran_velocity)

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
                if discontinuity_points is None:
                    # too close to edge
                    return

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

    def findDiscontinuityPoints(self) -> Optional[Tuple[Vec2, Vec2]]:
        """
        find the first and last points that are connected to the obstacle,
        in both the clockwise and counter-clockwise direction,
        if the drone is too close to either of them to avoid them,
        returns None.
        """

        obstacle = self.getBlockingObstacle(self.goal)

        # rotate points away from the obstacle,
        # such that the new point is on the tangent to the colision circle,
        # to avoid coliding on the obstacle,
        # when no furthur discontinuity points can be found
        cw = min(obstacle, key=lambda p: self.goal.angle(p))
        cw_avoidance_angle = getFoVCoverage(cw, self.boundary_distance)
        if cw_avoidance_angle is None:
            return None
        cw = cw.rotate(-cw_avoidance_angle)

        ccw = max(obstacle, key=lambda p: self.goal.angle(p))
        ccw_avoidance_angle = getFoVCoverage(ccw, self.boundary_distance)
        if ccw_avoidance_angle is None:
            return None
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
        #
        # include initial blocking obstacle in followed distance calculations,
        # to avoid going back and forth between boundary following and motion-to-goal.
        prev_followed_obstacle = [self.toWorldFrame(p)
                                  for p in self.getBlockingObstacle(self.goal)]

        while True:

            # ensure that the obstacle contains only points that are currently nearby
            followed_obstacle = set(self.toBodyFrame(p).round()
                                    for p in prev_followed_obstacle)
            followed_obstacle.intersection_update(p.round()
                                                  for p in self.nearby_points)

            # ensure that obstacles in the way to the followed obstalce are not ignored,
            followed_obstacle.update(
                p for p in self.nearby_points if p.length() < self.boundary_distance * 1.5)

            followed_point = min(followed_obstacle,
                                 key=lambda p: p.length(), default=None)

            if followed_point is None:
                # if the followed obstacle is unreachable, try motion to goal again
                return

            # add points near the new followed point to follow along in that direction
            # only adds points near the followed point,
            # to avoid staying in boundary following mode due to unreachable points.
            # the points should be on the side of the corridor being followed.
            followed_obstacle.update(
                self.getFollowedBoundary(followed_point))

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

            cur_followed_distance = min(p.distance(self.goal)
                                        for p in followed_obstacle)

            min_followed_distance = min(
                cur_followed_distance, min_followed_distance)

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
            assert avoidance_angle is not None
            rotated = point.rotate(avoidance_angle * angle_sign)

            # find the point that would avoid all other points on the obstacle as well
            angle = angle_sign * followed_point.angle(rotated)
            if max_angle < angle:
                max_angle = angle
                away_point = rotated

        return away_point
