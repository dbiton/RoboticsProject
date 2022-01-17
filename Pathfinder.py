import math

from DroneClient import *
from DroneTypes import *


def distance(p0: Position, p1: Position):
    return math.sqrt(pow(p0.x_m-p1.x_m, 2)+pow(p0.y_m-p1.y_m, 2)+pow(p0.z_m-p1.z_m, 2))

class Pathfinder:
    def __init__(self):
        self.client = None
        self.goal = None
        self.drone_speed = 0
        self.max_goal_distance = 0
        self.following_boundary = False
        self.min_boundary_distance = float("inf")
        self.leave_distance = None

    def setMaxGoalDistance(self, max_goal_distance: float):
        self.max_goal_distance = max_goal_distance

    def setDroneSpeed(self, drone_speed: float):
        self.drone_speed = drone_speed

    def setClient(self, client: DroneClient):
        self.client = client

    def setGoal(self, goal: Position):
        self.goal = goal

    def motionToGoal(self):
        self.following_boundary = False
        self.client.flyToPosition(self.goal.x_m, self.goal.y_m, self.goal.z_m, self.drone_speed)

    def followBoundary(self):
        self.following_boundary = True
        if self.min_boundary_distance > self.leave_distance:
            self.motionToGoal()

    def pathfind(self):
        pose = self.client.getPose()
        drone_pos = pose.pos
        goal_distance = distance(drone_pos, self.goal)
        if goal_distance < self.max_goal_distance:
            return True

        point_cloud = self.client.getLidarData()
        heuristic_distances = [distance(drone_pos, p) + distance(p, self.goal) for p in point_cloud]
        min_heuristic_distance = min(heuristic_distances)
        idx_min_heuristic_distance = heuristic_distances.index(min_heuristic_distance)
        self.leave_distance = min_heuristic_distance
        min_heuristic_point = point_cloud[idx_min_heuristic_distance]

        if self.following_boundary:
            self.followBoundary()
        else:
            self.motionToGoal()