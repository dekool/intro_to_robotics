from udpclient import RClient
import actions
import numpy as np
import math
import time
import map

states = ["ROTATE_TO_GOAL", "MOVE_STRAIGHT", "FOLLOW_OBJ", "DONE"]


class TangentBugs(object):
    def __init__(self, goal, robot_ip, update_map):
        self.RClient = RClient(robot_ip, 2777)
        self.goal = goal
        self.state = "ROTATE_TO_GOAL"
        self.old_sense = [0, 0, 0, 0, 0]
        self.current_position = [-999, -999]
        self.current_orientation = [0, 0]
        self.right_sense = -1
        self.front_sense = -1
        self.left_sense = -1
        self.object_start_dist = -1

        self.goal_tolerance = 8

        self.map = np.zeros([700, 380], dtype=int)
        self.update_map = update_map

    def update_parameters(self):
        counter = 0
        r_result = self.old_sense
        while r_result != self.old_sense or counter > 10:
            r_result = self.RClient.sense()  # call sense only once
            self.current_position = [r_result[0], r_result[1]]
            self.current_orientation = [r_result[2], r_result[3]]
            self.right_sense, self.front_sense, self.left_sense = r_result[4:]
            counter += 1
        if counter >= 10:
            print("ERROR - sensing haven't changed")

    def update_state(self):
        if ((round(self.current_position[0]) in np.arange(self.goal[0] - self.goal_tolerance, self.goal[0] + self.goal_tolerance)) and (
                round(self.current_position[1]) in np.arange(self.goal[1] - self.goal_tolerance, self.goal[1] + self.goal_tolerance))):
            self.state = "DONE"
        if self.state == "ROTATE_TO_GOAL":
            desired_orientation = [self.goal[0] - self.current_position[0], self.goal[1] - self.current_position[1]]
            norm = math.sqrt(desired_orientation[0] ** 2 + desired_orientation[1] ** 2)
            desired_orientation = [desired_orientation[0] / norm, desired_orientation[1] / norm]
            # find angle between orientations
            dot_product = np.matmul(self.current_orientation, desired_orientation)
            theta = np.arccos(dot_product) * 180 / math.pi
            if 0 < theta < 30:
                self.state = "MOVE_STRAIGHT"
        if self.state == "MOVE_STRAIGHT":
            if -1 < self.front_sense < 40:
                self.state = "FOLLOW_OBJ"
        if self.state == "FOLLOW_OBJ":
            if self.object_start_dist == -1:
                self.object_start_dist = math.sqrt((self.goal[0] - self.current_position[0]) ** 2 + (self.goal[1] - self.current_position[1]) ** 2)
            cur_dist = math.sqrt((self.goal[0] - self.current_position[0]) ** 2 + (self.goal[1] - self.current_position[1]) ** 2)
            if cur_dist < self.object_start_dist:
                self.object_start_dist = -1
                self.state = "ROTATE_TO_GOAL"

    def do_action(self):
        if self.state == "DONE":
            pass
        elif self.state == "ROTATE_TO_GOAL":
            actions.rotate_to_goal(self)
        elif self.state == "MOVE_STRAIGHT":
            actions.move_straight(self)
        elif self.state == "FOLLOW_OBJ":
            actions.follow_obj(self)

    def solve(self):
        if self.RClient.connect():
            while self.state != "DONE":
                self.update_parameters()
                if self.update_map:
                    map.save_points_to_map(self.map, self.RClient)
                self.update_state()
                self.do_action()
                time.sleep(0.1)

            # after done
            if self.update_map:
                map.draw_map()
            self.RClient.terminate()
        else:
            print("Failed to connect")


if __name__ == '__main__':
    algo = TangentBugs([0, 0], "192.168.1.152", update_map=True)
    algo.solve()
