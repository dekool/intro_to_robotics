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
        self.follow_obj_left_arr = []
        self.start_turning = False
        self.get_far_from_obj = False

        self.goal_tolerance = 5

        self.map = np.zeros([700, 380], dtype=int)
        self.update_map = update_map

    def update_parameters(self):
        counter = 0
        r_result = self.old_sense
        # while r_result != self.old_sense or counter > 10:
        while (r_result == self.old_sense or r_result[0] == -9999) and counter < 20:
            r_result = self.RClient.sense()  # call sense only once
            while len(r_result) < 6:
                r_result = self.RClient.sense()
            self.current_position = [r_result[0], r_result[1]]
            self.current_orientation = [r_result[2], r_result[3]]
            self.right_sense, self.front_sense, self.left_sense = r_result[4:]
            counter += 1
        self.old_sense = r_result
        if counter >= 10:
            print("ERROR - sensing haven't changed for 10 iterations")
        else:
            print("update_parameters | sensing... ", r_result)

    def update_state(self):
        print("update_state | curr state =", self.state)
        if ((round(self.current_position[0]) in np.arange(self.goal[0] - self.goal_tolerance,
                                                          self.goal[0] + self.goal_tolerance)) and (
                round(self.current_position[1]) in np.arange(self.goal[1] - self.goal_tolerance,
                                                             self.goal[1] + self.goal_tolerance))):
            self.state = "DONE"

        cur_dist = math.sqrt(
            (self.goal[0] - self.current_position[0]) ** 2 + (self.goal[1] - self.current_position[1]) ** 2)

        if self.state == "ROTATE_TO_GOAL":
            theta = self.calculate_theta()
            if 0 < theta < 25:
                self.state = "MOVE_STRAIGHT"
        if self.state == "MOVE_STRAIGHT":
            if -1 < self.front_sense < 40 and cur_dist > self.front_sense:
                # in case the goal is closer than the object - ignore the object
                self.follow_obj_left_arr = []
                self.start_turning = False
                self.get_far_from_obj = False
                self.state = "FOLLOW_OBJ"
            theta = self.calculate_theta()
            if theta > 25:
                self.state = "ROTATE_TO_GOAL"
        if self.state == "FOLLOW_OBJ":
            if self.object_start_dist == -1:
                self.object_start_dist = math.sqrt(
                    (self.goal[0] - self.current_position[0]) ** 2 + (self.goal[1] - self.current_position[1]) ** 2)
            cross_product = self.calculate_cross_product()
            if cur_dist < self.object_start_dist and (self.front_sense == -1 or self.front_sense > 70) and cross_product > 0:
                self.object_start_dist = -1
                self.state = "ROTATE_TO_GOAL"
            # in case of U type object - if the goal is closer than the object - ignore the object
            if cur_dist < self.left_sense:
                self.object_start_dist = -1
                self.state = "ROTATE_TO_GOAL"
        print("update_state | next state =", self.state)

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
                print("solve | updating parameters...")
                self.update_parameters()
                if self.update_map:
                    print("solve | updating map...")
                    map.save_points_to_map(self.map, self.RClient)
                print("solve | updating state...")
                self.update_state()
                print("solve | doing action...")
                self.do_action()
                time.sleep(0.5)

            # after done
            if self.update_map:
                map.draw_map(self.map, self.goal)
            self.RClient.terminate()
        else:
            print("Failed to connect")

    def calculate_theta(self):
        desired_orientation = [self.goal[0] - self.current_position[0], self.goal[1] - self.current_position[1]]
        norm = math.sqrt(desired_orientation[0] ** 2 + desired_orientation[1] ** 2)
        desired_orientation = [desired_orientation[0] / norm, desired_orientation[1] / norm]
        print("current position", self.current_position)
        print("update_state | current orientation = ", self.current_orientation, " desired_orientation = ",
              desired_orientation)
        # find angle between orientations
        dot_product = np.matmul(self.current_orientation, desired_orientation)
        theta = np.arccos(dot_product) * 180 / math.pi
        print("update_state | dot product = ", dot_product, " theta =", theta)
        return theta

    def calculate_cross_product(self):
        desired_orientation = [self.goal[0] - self.current_position[0],
                               self.goal[1] - self.current_position[1]]
        norm = math.sqrt(desired_orientation[0] ** 2 + desired_orientation[1] ** 2)
        desired_orientation = [desired_orientation[0] / norm, desired_orientation[1] / norm]
        cross_product = np.cross(self.current_orientation, desired_orientation)
        return cross_product


if __name__ == '__main__':
    algo = TangentBugs([100, -100], "192.168.1.156", update_map=True)
    algo.solve()
    print("MISSION ACCOMPLISHED - GOAL REACHED")
