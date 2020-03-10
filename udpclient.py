#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import socket
import json
import time
import errno
import threading
import matplotlib.pyplot as plt
from matplotlib import colors
from errnames import get_error_name
import math
import numpy as np

first = 1
right_obs_arr = []
front_obs_arr = []
left_obs_arr = []
map = np.zeros([700, 380], dtype=int)


def get_ip():
    from netifaces import interfaces, ifaddresses, AF_INET
    for interface in interfaces():
        try:
            for link in ifaddresses(interface)[AF_INET]:
                ip = str(link['addr'])
                if ip.startswith('192.168.1.'):
                    return ip
        except KeyError:
            pass
    return '127.0.0.1'


class RClient(object):
    """
	Robot python interface class
	Typical usage involves:

		r=RClient("192.168.1.152",2777)
		if not r.connect(): print error and exit
		while main_loop:
			r.drive(left_speed,right_speed)
			sensors=r.sense()
			some_calculations()
		r.terminate()

	"""

    def __init__(self, host, port, user_deprecate='', id_deprecate=''):
        self.ip = get_ip()
        # self.ip = host
        self.robot = (host, port)
        self.lock = threading.RLock()
        self.done = False
        self.sensors = [0.0, 0.0, 0.0, 0.0, 0.0]

    def connect(self):
        """ Connect to server and create processing thread """
        try:
            self.recv_thread = threading.Thread(target=self.recv_loop)
            self.recv_thread.start()
            return True
        except socket.error as e:
            reason = get_error_name(e.args[0])
            print("Socket Error: " + reason)
        return False

    def recv_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.ip, 9209))
        sock.setblocking(0)
        while not self.done:
            try:
                data, addr = sock.recvfrom(256)
                if len(data) == 0:
                    time.sleep(0.05)
                else:
                    # print("Received from '{}' data='{}'".format(addr, data))
                    try:
                        self.sensors = [float(s) for s in data.split()]
                    except ValueError:
                        pass
            except socket.error as e:
                # import pdb; pdb.set_trace()
                errnum = e.args[0]
                if errnum != errno.EAGAIN:
                    reason = get_error_name(errnum)
                # print("Socket Error ({}): {}".format(errnum,reason))
                time.sleep(0.5)

    def sendmsg(self, msg):
        msg = bytes(msg, "ascii")
        with self.lock:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.sendto(msg, self.robot)
                return True
            except socket.error:
                return False

    def terminate(self):
        """ Call before your program ends, for a clean exit """
        self.done = True
        self.recv_thread.join()

    def drive(self, left, right):
        """ Make the robot move.  Send 2 integers for motors [-1000 : 1000] """
        print(f"driving... {left} {right}")
        self.sendmsg('{} {}'.format(left, right))

    def sense(self):
        """ Get a list of sensor readings.  5 floating point values:  X,Y, 3 sonars """
        return self.sensors


#
#  Following code is a simple test main that allows to control the robot
#  from the keyboard, and see raw sensor readings on the screen
#
done = False
cmd = ''


def kbd():
    global cmd
    while cmd != 'q':
        cmd = sys.stdin.readline().strip()
        print("cmd = ", cmd)
    cmd = ''
    global done
    done = True


def move_manual(command, r):
    if len(command) == 2:
        try:
            r.drive(int(command[0]), int(command[1]))
        except ValueError:
            print("Invalid speeds")
    if len(command) == 1:
        if command[0] == 'w':
            r.drive(500, 500)
        elif command[0] == 'd':
            r.drive(0, 500)
        elif command[0] == 'a':
            r.drive(500, 0)
        elif command[0] == 's':
            r.drive(-500, -500)
        else:
            print("Invalid command for manual drive")


def move_straight(goal, r):  # assuming no obs
    tol = 8
    itr_cnt = 0
    current_position = [r.sense()[0], r.sense()[1]]
    print("move_straight: started. goal = ", goal, "current = ", current_position)
    while not ((round(current_position[0]) in np.arange(goal[0] - tol, goal[0] + tol)) and (
            round(current_position[1]) in np.arange(goal[1] - tol, goal[1] + tol))):
        current_orientation = [r.sense()[2], r.sense()[3]]
        desired_orientation = [goal[0] - r.sense()[0], goal[1] - r.sense()[1]]
        norm = math.sqrt(desired_orientation[0] ** 2 + desired_orientation[1] ** 2)
        desired_orientation = [desired_orientation[0] / norm, desired_orientation[1] / norm]
        print("move_straight: itr", itr_cnt, " |  current_orientation = ", current_orientation,
              " |  desired_orientation = ", desired_orientation)
        # find angle between orientations
        dot_product = np.matmul(current_orientation, desired_orientation)
        theta = np.arccos(dot_product) * 180 / math.pi
        cross_product = np.cross(current_orientation, desired_orientation)
        print("move_straight: itr", itr_cnt, " |  theta = ", theta, " |  cross_product = ",
              cross_product)
        while not(0 < theta < 30):
            # find rotation direction
            cross_product = np.cross(current_orientation, desired_orientation)
            print("move_straight: itr", itr_cnt, " |  theta = ", theta, " |  cross_product = ",
                  cross_product)

            # rotate robot
            # if 0 < theta < math.pi / 4:
            #     rot_speed = 0
            # else:
            #     rot_speed = (1000 / math.pi) * theta + 250  # find proportion empirically
            #     rot_speed = min(rot_speed, 1000)
            #     rot_speed = max(rot_speed, 350)
            rot_speed = 330
            if cross_product > 0:
                r.drive(rot_speed, -1 * rot_speed)
            else:
                r.drive(-1 * rot_speed, rot_speed)
            time.sleep(0.3)

            current_orientation = [r.sense()[2], r.sense()[3]]
            desired_orientation = [goal[0] - r.sense()[0], goal[1] - r.sense()[1]]
            norm = math.sqrt(desired_orientation[0] ** 2 + desired_orientation[1] ** 2)
            desired_orientation = [desired_orientation[0] / norm, desired_orientation[1] / norm]
            print("move_straight: itr", itr_cnt, " |  current_orientation = ", current_orientation,
                  " |  desired_orientation = ", desired_orientation)
            # find angle between orientations
            dot_product = np.matmul(current_orientation, desired_orientation)
            theta = np.arccos(dot_product) * 180 / math.pi
            print("move_straight: itr", itr_cnt, " |  theta = ", theta)

        right_obs = r.sense()[4]
        front_obs = r.sense()[5]
        left_obs = r.sense()[6]
        print("move_straight | distance to object: " + str(front_obs))
        if -1 < front_obs < 40:  # or -1 < right_obs < 40 or -1 < left_obs < 40:
            print("move_straight | front: ", front_obs, " left ", left_obs, "right: ", right_obs)
            print("move_straight | obstacle detected !")
            return
        # drive straight toward goal
        # higher_speed = 10 * norm  # find proportion empirically
        higher_speed = 450
        # higher_speed = min(higher_speed, 500)
        # higher_speed = max(higher_speed, 350)
        lower_speed = higher_speed * (math.pi / 2 - theta) / (math.pi / 2)
        lower_speed = min(lower_speed, 500)
        lower_speed = max(lower_speed, 350)
        if 40 < front_obs < 100:
            higher_speed = 300
            lower_speed = 300
        if cross_product > 0:
            r.drive(higher_speed, lower_speed)
        else:
            r.drive(lower_speed, higher_speed)

        print("move_straight: itr", itr_cnt, " |  higher_speed = ", higher_speed,
              " |  lower_speed = ", lower_speed)
        time.sleep(0.5)
        # update current position
        current_position = [r.sense()[0], r.sense()[1]]
        print("move_straight: itr", itr_cnt, " |  current_position = ", current_position)

        # itr_cnt += 1
        # if itr_cnt > 50:
        #     break

    print("move_straight: done. goal = ", goal, "current = ", current_position)


def pass_obj(r):
    turned = False
    while True:
        print(r.sense())
        if len(r.sense()) > 5:
            right_obs = r.sense()[4]
            front_obs = r.sense()[5]
            left_obs = r.sense()[6]
        else:
            front_obs = 1000
            right_obs = 1000
            left_obs = 1000
        print("pass_obj: right_obs =", right_obs, "| front_obs =", front_obs, "| left_obs =", left_obs)

        if turned and left_obs == -1 and right_obs == -1:
            return  # reached the end of the object
        else:
            if front_obs == -1 or front_obs > 100:
                r.drive(1000, 1000)
            if 0 < front_obs < 100:
                turned = True
                r.drive(350, -350)
                time.sleep(0.4)
        time.sleep(0.1)


def exp_joanne(r):
    global right_obs_arr, front_obs_arr, left_obs_arr
    x0 = x1 = x2 = 0
    right_obs = front_obs = left_obs = -1
    ### calculating constants ###
    p1 = [100, 200]  # obs_val, x0
    p2 = [30, 675]  # obs_val, x0
    m = (p1[1] - p2[1]) / (p1[0] - p2[0])
    n = p1[1] - m * p1[0]
    print("m = ", m, "| n = ", n)
    #############################
    print(r.sense())
    if len(r.sense()) > 5:
        right_obs = r.sense()[4]
        right_obs_arr = update_obs_arr(right_obs_arr, right_obs)
        right_sense_trend = calc_trend(right_obs_arr)
        front_obs = r.sense()[5]
        front_obs_arr = update_obs_arr(front_obs_arr, front_obs)
        front_sense_trend = calc_trend(front_obs_arr)
        left_obs = r.sense()[6]
        left_obs_arr = update_obs_arr(left_obs_arr, left_obs)
        left_sense_trend = calc_trend(left_obs_arr)
        print("right_obs_arr", right_obs_arr, "| front_obs_arr = ", front_obs_arr, "| left_obs_arr = ", left_obs_arr)
        print("right_sense_trend", right_sense_trend, "| front_sense_trend = ", front_sense_trend,
              "| left_sense_trend = ", left_sense_trend)
    #
    if front_sense_trend < 0:
        x0 = n + m * front_obs
    if right_sense_trend < 0:
        x1 = n + m * right_obs  # 100  # 1000 - 20 * right_obs
    if left_sense_trend < 0:
        x2 = n + m * left_obs  # 100  # 1000 - 20 * left_obs
    print("x0 = ", x0, "| x1 = ", x1, "| x2 = ", x2)
    right_speed = min(800 - x0 - x2, 1000)
    right_speed = max(800 - x0 - x2, 200)
    left_speed = min(800 - x0 - x1, 1000)
    left_speed = max(800 - x0 - x1, 200)
    print("right_speed = ", right_speed, "| left_speed = ", left_speed)
    r.drive(left_speed, right_speed)
    time.sleep(0.7)


def update_obs_arr(sense_arr, sense):
    max_arr_size = 5
    if sense != -1:
        if len(sense_arr) < max_arr_size:
            sense_arr.append(sense)
        else:
            np.roll(sense_arr, -1)
            sense_arr[-1] = sense
    else:
        sense_arr = []
    return sense_arr


def calc_trend(sense_arr):
    if len(sense_arr) > 1:
        diff = np.diff(sense_arr)
        trend = np.mean(diff)
    else:
        trend = 0
    return trend


def update_map(previous_sense, current_sense):
    previous_robot_position = previous_sense[0:2]
    previous_robot_orientation = previous_sense[2:4]
    robot_position = current_sense[0:2]
    robot_orientation = current_sense[2:4]

    if robot_position > [-1000, -1000] and previous_robot_position > [-1000, -1000]:  # ignore outliers
        plt.quiver(previous_robot_position[0], previous_robot_position[1], previous_robot_orientation[0],
                   previous_robot_orientation[1], color='r')
        plt.scatter(previous_robot_position[0], previous_robot_position[1], color='r')
        plt.quiver(robot_position[0], robot_position[1], robot_orientation[0], robot_orientation[1], color='g')
        plt.scatter(robot_position[0], robot_position[1], color='g')
    if len(current_sense) > 5:  # the first list received contains 5 elements only
        right_obs = current_sense[4]
        front_obs = current_sense[5]
        left_obs = current_sense[6]
        if front_obs != -1:
            plt.scatter(robot_position[0] + robot_orientation[0] * front_obs,
                        robot_position[1] + robot_orientation[1] * front_obs, color='k', marker='X')
        if right_obs != -1:
            plt.scatter(robot_position[0] + robot_orientation[0] * right_obs,
                        robot_position[1] - robot_orientation[1] * right_obs, color='k', marker='X')
        if left_obs != -1:
            plt.scatter(robot_position[0] - robot_orientation[0] * left_obs,
                        robot_position[1] + robot_orientation[1] * left_obs, color='k', marker='X')
    plt.pause(1.5)
    # plt.show()


def test():
    global done
    global cmd
    r = RClient("192.168.1.152", 2777)
    # r.drive(1000, 1000)
    counter = 0
    # initiate params
    front_obs = 1000
    right_obs = 1000
    left_obs = 1000
    turned = False
    current_sense = [0, 0, 0, 0, 0]
    goal = [0, -100]
    auto_drive = 0

    if r.connect():
        kbd_thread = threading.Thread(target=kbd)
        kbd_thread.start()
        while not done:
            # Keyboard input can be a pair of speeds, or q to exit
            if cmd:
                s = cmd.split()
                cmd = ''
                if s[0] == 'k':
                    print("auto drive is ON")
                    auto_drive = 1
                if s[0] == 'r':
                    save_points_to_map(r)
                move_manual(s, r)

            # move_straight(goal, r)
            time.sleep(0.1)
            counter += 1
            # if (counter%1000)==0:
            # done = True
            # print(r.sense())
            # print (len(r.sense()))
            previous_sense = current_sense
            current_sense = r.sense()
            # update_map(previous_sense, current_sense)
            if len(r.sense()) > 5:
                right_obs = r.sense()[4]
                front_obs = r.sense()[5]
                left_obs = r.sense()[6]

            # if front_obs > 0:
            #     pass_obj(r)
            if auto_drive == 1:
                # exp_joanne(r)
                # follow_obj(goal, r)
                algorithm(goal, r)
                break

        plt.show()
        r.terminate()
        print("Done")
        kbd_thread.join()
    else:
        print("Failed to connect")


def calculate_obj_point(r):
    r_result = r.sense()  # call sense only once
    current_position = r_result[0:2]
    current_orientation = [r_result[2], r_result[3]]
    right_sense, front_sense, left_sense = r_result[4:]
    front_obj_pos, left_obj_pos, right_obj_pos = -1, -1, -1
    if front_sense != -1:
        front_obj_pos = [current_position[0] + current_orientation[0] * front_sense,
                         current_position[1] + current_orientation[1] * front_sense]
    if left_sense != -1:
        # rotate by 45 degrees
        fixed_orientation = [((current_orientation[0] + current_orientation[1]) / (math.sqrt(2))),
                             ((current_orientation[1] - current_orientation[0]) / (math.sqrt(2)))]
        left_obj_pos = [current_position[0] + fixed_orientation[0] * left_sense,
                        current_position[1] + fixed_orientation[1] * left_sense]
    if right_sense != -1:
        # rotate by 45 degrees
        fixed_orientation = [((current_orientation[0] - current_orientation[1]) / (math.sqrt(2))),
                             ((current_orientation[0] + current_orientation[1]) / (math.sqrt(2)))]
        right_obj_pos = [current_position[0] + fixed_orientation[0] * right_sense,
                         current_position[1] + fixed_orientation[1] * right_sense]
    print("left obj: " + str(left_obj_pos) + " front obj: " + str(front_obj_pos) + " right obj: " + str(right_obj_pos))
    print("robot position: " + str(current_position))
    return left_obj_pos, front_obj_pos, right_obj_pos


def save_points_to_map(r):
    """
    boundries:
    [-237.293, 93.4245]
    [435.734, 81.745]
    [153.742, -275.195]
    [-46.4623, -277.982]

    0 - unknown
    1 - clear
    2 - object
    """
    X_LOWER_FIX = 240
    Y_LOWER_FIX = 280
    left_obj_pos, front_obj_pos, right_obj_pos = calculate_obj_point(r)
    r_result = r.sense()  # call sense only once
    current_position = r_result[0:2]

    # current position is empty
    current_position = [math.floor(current_position[0]), math.floor(current_position[1])]
    map[current_position[0] + X_LOWER_FIX, current_position[1] + Y_LOWER_FIX] = 1
    map[current_position[0] + 1 + X_LOWER_FIX, current_position[1] + Y_LOWER_FIX] = 1
    map[current_position[0] + X_LOWER_FIX, current_position[1] + 1 + Y_LOWER_FIX] = 1
    map[current_position[0] + 1 + X_LOWER_FIX, current_position[1] + 1 + Y_LOWER_FIX] = 1

    if left_obj_pos != -1:
        left_obj_pos = [math.floor(left_obj_pos[0]), math.floor(left_obj_pos[1])]
        map[left_obj_pos[0] + X_LOWER_FIX, left_obj_pos[1] + Y_LOWER_FIX] = 2
        # we can add else - 100 units are clear
    if front_obj_pos != -1:
        front_obj_pos = [math.floor(front_obj_pos[0]), math.floor(front_obj_pos[1])]
        map[front_obj_pos[0] + X_LOWER_FIX, front_obj_pos[1] + Y_LOWER_FIX] = 2
    if right_obj_pos != -1:
        right_obj_pos = [math.floor(right_obj_pos[0]), math.floor(right_obj_pos[1])]
        map[right_obj_pos[0] + X_LOWER_FIX, right_obj_pos[1] + Y_LOWER_FIX] = 2


def draw_map():
    cmap = colors.ListedColormap(['white', 'Blue', 'red'])
    plt.figure()
    plt.pcolor(map, cmap=cmap, edgecolors='k', linewidth=0.05)
    plt.show()


def goal_reached(goal, current_position, tol):
    return ((round(current_position[0]) in np.arange(goal[0] - tol, goal[0] + tol)) and (
            round(current_position[1]) in np.arange(goal[1] - tol, goal[1] + tol)))


def stop_following_obj(goal, current_position, obj_start_pos):
    cur_dist = math.sqrt((goal[0] - current_position[0]) ** 2 + (goal[1] - current_position[1]) ** 2)
    start_dist = math.sqrt((goal[0] - obj_start_pos[0]) ** 2 + (goal[1] - obj_start_pos[1]) ** 2)


def fix_to_parallel(r):
    print("fix_to_parallel | START")
    r_result = r.sense()  # call sense only once
    right_sense, front_sense, left_sense = r_result[4:]
    print("fix_to_parallel | sensing...")
    print("fix_to_parallel | right_obs =", right_sense, "| front_obs =", front_sense, "| left_obs =", left_sense)
    while front_sense != -1 and front_sense < 70:
        if right_sense < left_sense:
            theta = math.atan((front_sense * math.sin(135)) / (right_sense + front_sense * math.cos(135)))
        else:
            theta = math.atan((front_sense * math.sin(135)) / (left_sense + front_sense * math.cos(135)))

        print("fix_to_parallel | theta = ", (theta * 180 / math.pi))
        # rotate robot
        # rot_speed = (1000 / math.pi) * theta + 250  # find proportion empirically
        # rot_speed = min(rot_speed, 1000)
        # rot_speed = max(rot_speed, 300)
        rot_speed = 320
        print("fix_to_parallel | rotate robot...")
        # print("fix_to_parallel | rot speed: ", rot_speed)
        r.drive(rot_speed, -1 * rot_speed)
        time.sleep(0.4)

        r_result = r.sense()  # call sense only once
        right_sense, front_sense, left_sense = r_result[4:]
        print("fix_to_parallel | sensing...")
        print("fix_to_parallel | right_obs =", right_sense, "| front_obs =", front_sense, "| left_obs =", left_sense)

    print("fix_to_parallel | DONE")


def fix_to_orthogonal(r, edge_of_obj):
    rot_speed = 350
    if edge_of_obj == -1:
        return
    r_result = r.sense()
    current_position = [r_result[0], r_result[1]]
    current_orientation = [r_result[2], r_result[3]]
    radius_orientation = [edge_of_obj[0] - current_position[0], edge_of_obj[1] - current_position[1]]
    norm = math.sqrt(radius_orientation[0] ** 2 + radius_orientation[1] ** 2)
    radius_orientation = [radius_orientation[0] / norm, radius_orientation[1] / norm]
    dot_product = np.matmul(current_orientation, radius_orientation)
    old_theta = -1
    theta = np.arccos(dot_product) * 180 / math.pi
    print("fix_to_orthogonal | current_position = ", current_position)
    print("fix_to_orthogonal | current_orientation = ", current_orientation)
    print("fix_to_orthogonal | radius_orientation = ", radius_orientation)
    print("fix_to_orthogonal | dot_product = ", dot_product, " theta = ", theta)
    while theta < 70 or theta > 110:
        if old_theta != theta:
            r.drive(rot_speed, -1 * rot_speed)
        print("fix_to_orthogonal | fixing orientation...")
        r_result = r.sense()
        current_position = [r_result[0], r_result[1]]
        current_orientation = [r_result[2], r_result[3]]
        radius_orientation = [edge_of_obj[0] - current_position[0], edge_of_obj[1] - current_position[1]]
        norm = math.sqrt(radius_orientation[0] ** 2 + radius_orientation[1] ** 2)
        radius_orientation = [radius_orientation[0] / norm, radius_orientation[1] / norm]
        dot_product = np.matmul(current_orientation, radius_orientation)
        old_theta = theta
        theta = np.arccos(dot_product) * 180 / math.pi
        print("fix_to_orthogonal | current_position = ", current_position)
        print("fix_to_orthogonal | current_orientation = ", current_orientation)
        print("fix_to_orthogonal | radius_orientation = ", radius_orientation)
        print("fix_to_orthogonal | dot_product = ", dot_product, " theta = ", theta)
        time.sleep(0.4)


def move_parallel_to_obj(r):
    print("move_parallel_to_obj | START")
    r_result = r.sense()  # call sense only once
    right_sense, front_sense, left_sense = r_result[4:]
    print("move_parallel_to_obj | sensing...")
    print("move_parallel_to_obj | right_obs =", right_sense, "| front_obs =", front_sense, "| left_obs =", left_sense)
    edge_of_obj = -1
    while left_sense != -1:
        if front_sense != -1 and front_sense < 70:
            fix_to_parallel(r)
        else:
            r.drive(400, 400)
        time.sleep(0.5)

        r_result = r.sense()  # call sense only once
        right_sense, front_sense, left_sense = r_result[4:]
        if left_sense != -1:
            edge_of_obj = calculate_obj_point(r)[0]
        print("move_parallel_to_obj | sensing...")
        print("move_parallel_to_obj | right_obs =", right_sense, "| front_obs =", front_sense, "| left_obs =",
              left_sense)

    print("move_parallel_to_obj | DONE")
    return edge_of_obj


def turn_around_obj(r, edge_of_obj):
    print("turn_around_obj | START")
    r_result = r.sense()  # call sense only once
    right_sense, front_sense, left_sense = r_result[4:]
    print("turn_around_obj | sensing...")
    print("turn_around_obj | right_obs =", right_sense, "| front_obs =", front_sense, "| left_obs =", left_sense)
    start_turn_orientation = [r_result[2], r_result[3]]
    current_orientation = [r_result[2], r_result[3]]

    # find angle between orientations
    print("turn_around_obj | start_turn_orientation = ", start_turn_orientation, " current_orientation = ",
          current_orientation)
    # dot_product = np.matmul(start_turn_orientation, current_orientation)
    # theta = np.arccos(dot_product)
    # theta = theta * 180 / math.pi  # convert to degrees
    theta = 0
    # print("turn_around_obj | dot_product = ", dot_product, " theta = ", theta)
    #

    while theta < 150:
        # print("turn_around_obj | right_obs =", right_sense, "| front_obs =", front_sense, "| left_obs =", left_sense)
        if front_sense != -1:
            print("turn_around_obj | new obj!!!")  # need to consider what to do
            return "obj"
        # fix_to_orthogonal(r, edge_of_obj)
        r.drive(0, 400)  # slow turn
        # if left_sense != -1:
        if -1 < left_sense < 40:
            r.drive(400, 0)  # fix orientation
        else:
            r.drive(0, 400)  # slow turn
        time.sleep(0.3)
        # calculate details again
        r_result = r.sense()  # call sense only once
        right_sense, front_sense, left_sense = r_result[4:]
        current_orientation = [r_result[2], r_result[3]]
        if start_turn_orientation == current_orientation:
            r_result = r.sense()  # call sense only once
            right_sense, front_sense, left_sense = r_result[4:]
            current_orientation = [r_result[2], r_result[3]]
        print("turn_around_obj | start_turn_orientation = ", start_turn_orientation, " current_orientation = ",
              current_orientation)

        # find angle between orientations
        if start_turn_orientation == current_orientation:
            dot_product = 1
            theta = 0
        else:
            dot_product = np.matmul(start_turn_orientation, current_orientation)
            if dot_product > 1:
                theta = 0
            else:
                theta = np.arccos(dot_product)
            theta = theta * 180 / math.pi  # convert to degrees
        print("turn_around_obj | dot_product = ", dot_product, " theta = ", theta)
        print("turn_around_obj | right_obs =", right_sense, "| front_obs =", front_sense, "| left_obs =", left_sense)

    print("turn_around_obj | DONE")
    return "ok"


def follow_obj(goal, r, obj_start_pos=None):
    print("follow_obj | START")
    r_result = r.sense()  # call sense only once
    current_position = r_result[0:2]
    right_sense, front_sense, left_sense = r_result[4:]
    if obj_start_pos is None:
        obj_start_pos = current_position
    else:
        cur_dist = math.sqrt((goal[0] - current_position[0]) ** 2 + (goal[1] - current_position[1]) ** 2)
        start_dist = math.sqrt((goal[0] - obj_start_pos[0]) ** 2 + (goal[1] - obj_start_pos[1]) ** 2)
        if cur_dist < start_dist:
            print("follow_obj | END")
            return
    start_dist = math.sqrt((goal[0] - obj_start_pos[0]) ** 2 + (goal[1] - obj_start_pos[1]) ** 2)
    print("follow_obj | goal : ", goal[0], goal[1])
    print("follow_obj | first sensing...")
    print("follow_obj | obj_start_pos : ", obj_start_pos[0], obj_start_pos[1])
    print("follow_obj | right_obs =", right_sense, "| front_obs =", front_sense, "| left_obs =", left_sense)
    print("follow_obj | start distance to goal =  ", start_dist)
    fix_to_parallel(r)
    # print("follow_obj | right_obs =", right_sense, "| front_obs =", front_sense, "| left_obs =", left_sense)
    # print("follow_obj | done first while (fix_to_parallel)")
    first_left_sense = left_sense
    time.sleep(1)
    edge_of_obj = move_parallel_to_obj(r)

    # pass the obj
    time.sleep(1)
    res = turn_around_obj(r, edge_of_obj)
    print("res of turn around obj: " + str(res))
    if res == "obj":
        return obj_start_pos

    # start calculating distance
    r_result = r.sense()
    current_position = r_result[0:2]
    cur_dist = math.sqrt((goal[0] - current_position[0]) ** 2 + (goal[1] - current_position[1]) ** 2)
    print("follow_obj | start dist: " + str(start_dist))
    print("follow_obj | curr dist: " + str(cur_dist))
    counter = 0
    while cur_dist > start_dist and counter < 6:
        if front_sense != -1 and front_sense < 500:
            fix_to_parallel(r)
        r.drive(400, 400)
        time.sleep(0.4)
        if left_sense == -1:
            counter += 1
            r.drive(0, 400)
        r_result = r.sense()  # call sense only once
        right_sense, front_sense, left_sense = r_result[4:]
        print("follow_obj | right_obs =", right_sense, "| front_obs =", front_sense, "| left_obs =", left_sense)
        current_position = r_result[0:2]
        cur_dist = math.sqrt((goal[0] - current_position[0]) ** 2 + (goal[1] - current_position[1]) ** 2)
        print("follow_obj | curr dist: " + str(cur_dist))

    print("follow_obj | reached the point")
    print("follow_obj | DONE")
    return None


def algorithm(goal, r):
    sensing = r.sense()
    while len(sensing) < 6:
        sensing = r.sense()
    current_position = [r.sense()[0], r.sense()[1]]
    last_obj_start_pos = None
    while not goal_reached(goal, current_position, tol=8):
        if last_obj_start_pos is None:
            move_straight(goal, r)
        sensing = r.sense()
        right_obs = sensing[4]
        front_obs = sensing[5]
        left_obs = sensing[6]
        # if (-1 < front_obs < 40 or -1 < right_obs < 40 or -1 < left_obs < 40) or last_obj_start_pos is not None:
        if (-1 < front_obs < 40) or last_obj_start_pos is not None:
            last_obj_start_pos = follow_obj(goal, r, last_obj_start_pos)
        current_position = [r.sense()[0], r.sense()[1]]


if __name__ == '__main__':
    test()

    # robot_position = np.array([[1, 2], [2, 3], [3, 2], [2, 1]])
    # robot_orientation = np.array([[1, 1], [1, -1], [-1, -1], [-1, 1]])
    #
    # for i in range(4):
    #     if i != 0:
    #         plt.quiver(robot_position[i - 1, 0], robot_position[i - 1, 1], robot_orientation[i - 1, 0],
    #                    robot_orientation[i - 1, 1], color='r')
    #         plt.scatter(robot_position[i - 1, 0], robot_position[i - 1, 1], color='r')
    #     plt.quiver(robot_position[i, 0], robot_position[i, 1], robot_orientation[i, 0], robot_orientation[i, 1],
    #                color='g')
    #     plt.scatter(robot_position[i, 0], robot_position[i, 1], color='g')
    #     plt.pause(1.5)
    #
    # plt.show()
