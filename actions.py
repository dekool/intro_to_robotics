from tangent_bugs import TangentBugs
import math
import numpy as np
import time


def rotate_to_goal(tangentBugs):
    if tangentBugs.current_position[0] == -9999:
        tangentBugs.RClient.drive(300, 300)
        return
    desired_orientation = [tangentBugs.goal[0] - tangentBugs.current_position[0], tangentBugs.goal[1] - tangentBugs.current_position[1]]
    norm = math.sqrt(desired_orientation[0] ** 2 + desired_orientation[1] ** 2)
    desired_orientation = [desired_orientation[0] / norm, desired_orientation[1] / norm]
    cross_product = np.cross(tangentBugs.current_orientation, desired_orientation)

    rot_speed = 350
    if cross_product > 0:
        tangentBugs.RClient.drive(rot_speed, -1 * rot_speed)
    else:
        tangentBugs.RClient.drive(-1 * rot_speed, rot_speed)


def move_straight(tangentBugs):
    cur_dist = math.sqrt(
        (tangentBugs.goal[0] - tangentBugs.current_position[0]) ** 2 + (tangentBugs.goal[1] - tangentBugs.current_position[1]) ** 2)
    if -1 < tangentBugs.front_sense < 40 and cur_dist > tangentBugs.front_sense:  # or -1 < right_obs < 40 or -1 < left_obs < 40:
        print("move_straight | front: ", tangentBugs.front_sense, " left ", tangentBugs.left_sense, "right: ", tangentBugs.right_sense)
        print("move_straight | obstacle detected !")
        return
    if tangentBugs.current_position[0] == -9999:
        tangentBugs.RClient.drive(300, 300)
        return
    # drive straight toward goal
    if 0 < tangentBugs.left_sense < 30 and cur_dist > 50:
        tangentBugs.RClient.drive(500, 0)
        time.sleep(0.3)
    elif 0 < tangentBugs.right_sense < 30 and cur_dist > 50:
        tangentBugs.RClient.drive(0, 500)
        time.sleep(0.3)
    desired_orientation = [tangentBugs.goal[0] - tangentBugs.current_position[0],
                           tangentBugs.goal[1] - tangentBugs.current_position[1]]
    norm = math.sqrt(desired_orientation[0] ** 2 + desired_orientation[1] ** 2)
    desired_orientation = [desired_orientation[0] / norm, desired_orientation[1] / norm]
    cross_product = np.cross(tangentBugs.current_orientation, desired_orientation)
    # find angle between orientations
    dot_product = np.matmul(tangentBugs.current_orientation, desired_orientation)
    theta = np.arccos(dot_product) * 180 / math.pi
    print("move_straight | theta = ", theta)
    # higher_speed = 10 * norm  # find proportion empirically
    higher_speed = 450
    # lower_speed = higher_speed * (math.pi / 2 - theta) / (math.pi / 2)
    lower_speed = higher_speed * (90 - theta) / 90
    lower_speed = min(lower_speed, higher_speed)
    lower_speed = max(lower_speed, 350)
    if 0 < tangentBugs.front_sense < 100 or cur_dist < 50:
        higher_speed = 300
        lower_speed = 300
    if cross_product > 0:
        tangentBugs.RClient.drive(higher_speed, lower_speed)
    else:
        tangentBugs.RClient.drive(lower_speed, higher_speed)


def follow_obj(tangentBugs):
    if tangentBugs.front_sense != -1 and tangentBugs.front_sense < 70:
        if tangentBugs.front_sense < 25:  # take small step backwards
            print("follow_obj | too close to object from front...")
            tangentBugs.RClient.drive(-300, -300)
        else:
            rot_speed = 350
            print("fix_to_parallel | rotate robot...")
            tangentBugs.RClient.drive(rot_speed, -1 * rot_speed)
    else:
        if tangentBugs.left_sense == -1:
            if tangentBugs.start_turning:  # one more drive straight to ovoid being too close to the edge
                tangentBugs.start_turning = False
                print("Start turning")
                tangentBugs.RClient.drive(350, 350)
            else:
                tangentBugs.follow_obj_left_arr = []
                tangentBugs.RClient.drive(0, 400)  # turn or fix
        else:
            tangentBugs.follow_obj_left_arr.append(tangentBugs.left_sense)
            if (tangentBugs.left_sense < 25 or
                (len(tangentBugs.follow_obj_left_arr) > 2
                 and tangentBugs.follow_obj_left_arr[-1] < tangentBugs.follow_obj_left_arr[-2] + 10)) \
                    and tangentBugs.get_far_from_obj is False:
                rot_speed = 330
                print("follow_obj | too close to object from left...")
                # tangentBugs.RClient.drive(-1 * rot_speed, rot_speed)
                tangentBugs.RClient.drive(500, 0)
                tangentBugs.get_far_from_obj = True
            else:
                tangentBugs.RClient.drive(350, 350)
                tangentBugs.start_turning = True
                tangentBugs.get_far_from_obj = False
