from operator import itemgetter
import math

from DroneClient import *
from DroneTypes import *


def cartesianToPolar(p: (float, float)):
    return math.sqrt(p[0] ** 2 + p[1] ** 2), math.atan2(p[1], p[0])


def polarToCartesian(p: (float, float)):
    return p[0] * math.cos(p[1]), p[0] * math.sin(p[1])


def distance(p0: (float, float), p1: (float, float)):
    return math.sqrt((p0[0] - p1[0]) ** 2 + (p0[1] - p1[1]) ** 2)


def vector(src: (float, float), dst: (float, float)):
    return dst[0] - src[0], dst[1] - dst[1]


class TangentBug:
    def __init__(self):
        self.prev_heuristic_distance = float("inf")
        self.is_following_boundary = False
        self.goal = (0, 0)
        self.segment_rot_epsilon = 0.01
        self.segment_len_epsilon = 1
        self.goal_distance_epsilon = 1

    def setGoal(self, goal: (float, float)):
        self.goal = goal

    def calcEdgePoints(self, polar_points: list):
        edges = []
        for i in range(len(polar_points)):
            p0 = polar_points[i - 1]
            p1 = polar_points[i]
            if math.abs(p0[0] - p1[0]) > self.segment_len_epsilon or math.abs(p0[1] - p1[1]) > self.segment_rot_epsilon:
                edges.append(polarToCartesian(p0))
        return edges

    def goalPathIntersectsSegment(self, pos: (float, float), polar_points: list):
        angle = math.atan2(self.goal[1] - pos[1], self.goal[0] - pos[0])
        pps_sorted = polar_points.sorted(key=math.abs(itemgetter(1) - angle))
        p = pps_sorted[0]
        if math.abs(p[1] - angle) < self.segment_rot_epsilon:
            return distance(pos, self.goal) > p[0]
        else:
            return False

    def motionToGoal(self, pos, points):
        polar_points = [cartesianToPolar(p) for p in points]
        polar_points.sort(key=itemgetter(1))
        if self.goalPathIntersectsSegment(pos, polar_points):
            edge_points = self.calcEdgePoints(polar_points)
            heuristic_distances = [distance(pos, o) + distance(o, self.goal) for o in edge_points]
            min_heuristic_distance = min(heuristic_distances)
            if min_heuristic_distance < self.prev_heuristic_distance:
                self.prev_heuristic_distance = min_heuristic_distance
                i = heuristic_distances.index(min_heuristic_distance)
                edge_point = edge_points[i]
                return vector(pos, edge_point)
            else:
                self.prev_heuristic_distance = float("inf")
                self.is_following_boundary = True
                return self.followBoundary(pos, points)
        else:
            return vector(pos, self.goal)

    def followBoundary(self, pos, points: list):
        point_distances = [distance(pos, p) for p in points]
        i = point_distances.index(min(point_distances))
        boundary_point = points[i]
        reach_distance = distance(pos, goal)
        followed_distance = distance(boundary_point, goal)
        if reach_distance >= followed_distance:
            boundary_normal = vector(boundary_point, pos)
            boundary_tangent = (boundary_normal[1], -boundary_normal[0])
            return pos[0]+boundary_tangent[0], pos[1]+boundary_tangent[1]
        else:
            self.is_following_boundary = False
            return self.motionToGoal(pos, points)

    def pathfind(self, pos: (float, float), points: list):
        if distance(pos, goal) < self.goal_distance_epsilon:
            return pos
        if self.is_following_boundary:
            return self.followBoundary(pos, points)
        else:
            return self.motionToGoal(pos, points)
