import math
import matplotlib.pyplot as plt
from matplotlib import colors

X_LOWER_FIX = 240
Y_LOWER_FIX = 280


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


def save_points_to_map(map, r):
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

    left_obj_pos, front_obj_pos, right_obj_pos = calculate_obj_point(r)
    r_result = r.sense()  # call sense only once
    current_position = r_result[0:2]
    if current_position[0] == -9999:
        print("out of boundry - can't save")
        return

    # current position is empty
    current_position = [math.floor(current_position[0]), math.floor(current_position[1])]
    map[current_position[0] + X_LOWER_FIX, current_position[1] + Y_LOWER_FIX] = 1
    map[current_position[0] + 1 + X_LOWER_FIX, current_position[1] + Y_LOWER_FIX] = 1
    map[current_position[0] + X_LOWER_FIX, current_position[1] + 1 + Y_LOWER_FIX] = 1
    map[current_position[0] + 1 + X_LOWER_FIX, current_position[1] + 1 + Y_LOWER_FIX] = 1

    if left_obj_pos != -1:
        left_obj_pos = [math.floor(left_obj_pos[0]), math.floor(left_obj_pos[1])]
        map[left_obj_pos[0] + X_LOWER_FIX, left_obj_pos[1] + Y_LOWER_FIX] = 2
        map[left_obj_pos[0] + X_LOWER_FIX + 1, left_obj_pos[1] + Y_LOWER_FIX] = 2
        map[left_obj_pos[0] + X_LOWER_FIX, left_obj_pos[1] + Y_LOWER_FIX + 1] = 2
        map[left_obj_pos[0] + X_LOWER_FIX + 1, left_obj_pos[1] + Y_LOWER_FIX + 1] = 2
        # we can add else - 100 units are clear
    if front_obj_pos != -1:
        front_obj_pos = [math.floor(front_obj_pos[0]), math.floor(front_obj_pos[1])]
        map[front_obj_pos[0] + X_LOWER_FIX, front_obj_pos[1] + Y_LOWER_FIX] = 2
        map[front_obj_pos[0] + X_LOWER_FIX + 1, front_obj_pos[1] + Y_LOWER_FIX] = 2
        map[front_obj_pos[0] + X_LOWER_FIX, front_obj_pos[1] + Y_LOWER_FIX + 1] = 2
        map[front_obj_pos[0] + X_LOWER_FIX + 1, front_obj_pos[1] + Y_LOWER_FIX + 1] = 2
    if right_obj_pos != -1:
        right_obj_pos = [math.floor(right_obj_pos[0]), math.floor(right_obj_pos[1])]
        map[right_obj_pos[0] + X_LOWER_FIX, right_obj_pos[1] + Y_LOWER_FIX] = 2
        map[right_obj_pos[0] + X_LOWER_FIX + 1, right_obj_pos[1] + Y_LOWER_FIX] = 2
        map[right_obj_pos[0] + X_LOWER_FIX, right_obj_pos[1] + Y_LOWER_FIX + 1] = 2
        map[right_obj_pos[0] + X_LOWER_FIX + 1, right_obj_pos[1] + Y_LOWER_FIX + 1] = 2


def save_boundries_to_map(map):
    for x in range(-50, 155):
        map[x + X_LOWER_FIX, -280 + Y_LOWER_FIX] = 2
    for x in range(-240, 440):
        map[x + X_LOWER_FIX, 99 + Y_LOWER_FIX] = 2

    point_1 = [155, -280]
    point_2 = [440, 100]
    m = (point_2[1] - point_1[1]) / (point_2[0] - point_1[0])
    n = point_1[1] - m * point_1[0]
    for x in range(155, 440):
        y = x * m + n
        map[x + X_LOWER_FIX, math.floor(y + Y_LOWER_FIX)] = 2
    point_1 = [-240, 100]
    point_2 = [-50, -280]
    m = (point_2[1] - point_1[1]) / (point_2[0] - point_1[0])
    n = point_1[1] - m * point_1[0]
    for x in range(-240, -50):
        y = x * m + n
        map[x + X_LOWER_FIX, min(math.floor(y + Y_LOWER_FIX), 379)] = 2


def close_to_left_wall(current_position):
    point_1 = [155, -280]
    point_2 = [440, 100]
    m = (point_2[1] - point_1[1]) / (point_2[0] - point_1[0])
    n = point_1[1] - m * point_1[0]
    distance_from_wall = abs(m*current_position[0] - current_position[1] + n) / (math.sqrt(m**2 + 1))
    if distance_from_wall < 10:
        return True


def close_to_right_wall(current_position):
    point_1 = [-240, 100]
    point_2 = [-50, -280]
    m = (point_2[1] - point_1[1]) / (point_2[0] - point_1[0])
    n = point_1[1] - m * point_1[0]
    distance_from_wall = abs(m*current_position[0] - current_position[1] + n) / (math.sqrt(m**2 + 1))
    if distance_from_wall < 10:
        return True


def check_close_to_boundries(current_position):
    back_wall_Y = -280
    front_wall_Y = 100
    if close_to_left_wall(current_position) or close_to_right_wall(current_position) or \
            current_position[1] < (back_wall_Y + 10) or (front_wall_Y - 10) < current_position[1]:
        return True


def draw_map(world_map):
    save_boundries_to_map(world_map)
    cmap = colors.ListedColormap(['white', 'Blue', 'red'])
    plt.figure()
    plt.pcolor(world_map, cmap=cmap, edgecolors='k', linewidth=0.05)
    plt.show()
