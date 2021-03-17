from enums import Cell, Direction, Movement
from utils import print_map, generate_unexplored_map
from map_descriptor import generate_map, generate_map_descriptor_for_android
from fastest_path.fastest_path import FastestPath
from constants import START_POS, GOAL_POS, NUM_ROWS, NUM_COLS
from robots import SimulatorBot
import time
from collections import deque


MIN_STEPS_WITHOUT_CALIBRATION = 3


class Exploration:
    def __init__(self, robot, on_update_map=None, on_calibrate=None, explored_map=None, coverage_limit=None,
                 time_limit=None):
        """
        Args:
            robot (robots.Robot): Robot object to explore the map.
            on_update_map (function): Function called whenever the map is updated.
            explored_map (list): 2D list of Cell objects representing the map.
            coverage_limit (int): Coverage limit as a % (e.g. 100 means 100%).
            time_limit (int): Time limit in seconds.
        """
        self.robot = robot
        self.entered_goal = False
        self.prev_pos = None
        self.explored_map = explored_map if explored_map is not None else generate_unexplored_map()
        self.start_time = time.time()
        self.coverage_limit = coverage_limit
        self.time_limit = time_limit
        self.steps_without_calibration = 0
        self.obstacles = {}
        self.is_running = True
        self.queue = deque([])

        if on_update_map is None:
            self.on_update_map = lambda: None
        else:
            self.on_update_map = on_update_map

        if on_calibrate is None:
            self.on_calibrate = lambda is_front: None
        else:
            self.on_calibrate = on_calibrate

    @property
    def coverage(self):
        return 1 - (sum([row.count(Cell.UNEXPLORED) for row in self.explored_map]) / (NUM_ROWS * NUM_COLS))

    @property
    def time_elapsed(self):
        return time.time() - self.start_time

    @property
    def is_limit_exceeded(self):
        is_coverage_limit_exceeded = self.coverage_limit is not None and self.coverage_limit < self.coverage
        is_time_limit_exceeded = self.time_limit is not None and self.time_limit < \
                                 self.time_elapsed + (FastestPath.heuristic_function(self.robot.pos,
                                                                                     START_POS) * 2) / self.robot.speed

        return not self.is_running or is_coverage_limit_exceeded or is_time_limit_exceeded

    # find cord wrt the bot based on where it's facing
    def find_right_pos(self):
        direction_vector = Direction.get_direction_vector((self.robot.direction + 1) % 4)
        current_pos = self.robot.pos
        return current_pos[0] + direction_vector[0], current_pos[1] + direction_vector[1]

    # check for surroundings
    def check_surroundings(self, movement):
        check_direction = self.robot.direction

        if movement == Movement.RIGHT:
            check_direction = (check_direction + 1) % 4
        elif movement == Movement.LEFT:
            check_direction = (check_direction - 1) % 4
        elif movement == Movement.BACKWARD:
            check_direction = (check_direction + 2) % 4

        if check_direction == Direction.NORTH:
            return (0, 2), (-1, 2), (1, 2)
        elif check_direction == Direction.EAST:
            return (2, 0), (2, -1), (2, 1)
        elif check_direction == Direction.SOUTH:
            return (0, -2), (-1, -2), (1, -2)
        elif check_direction == Direction.WEST:
            return (-2, 0), (-2, 1), (-2, -1)

    # check if should turn right
    def check_right(self):
        current_pos = self.robot.pos
        obstacle = False

        for x, y in self.check_surroundings(Movement.RIGHT):
            # check wall
            if current_pos[1] + y < 0 or current_pos[1] + y > 19 or current_pos[0] + x < 0 or current_pos[0] + x > 14:
                return False
            # check if right is unexplored
            elif self.explored_map[current_pos[1] + y][current_pos[0] + x] == Cell.UNEXPLORED:
                return True
            # check for obstacles
            elif self.explored_map[current_pos[1] + y][current_pos[0] + x] == Cell.OBSTACLE:
                obstacle = True

        if obstacle:
            return False

        right_pos = self.find_right_pos()
        if right_pos == self.prev_pos:
            return False
        return True

    # check if should move forward
    def check_forward(self):
        current_pos = self.robot.pos

        for x, y in self.check_surroundings(Movement.FORWARD):
            # check wall
            if current_pos[1] + y < 0 or current_pos[1] + y > 19 or current_pos[0] + x < 0 or current_pos[0] + x > 14:
                return False
            # check for obstacles
            elif self.explored_map[current_pos[1] + y][current_pos[0] + x] == Cell.OBSTACLE:
                return False

        return True

    # check if should turn left
    def check_left(self):
        current_pos = self.robot.pos

        for x, y in self.check_surroundings(Movement.LEFT):
            # check wall
            if current_pos[1] + y < 0 or current_pos[1] + y > 19 or current_pos[0] + x < 0 or current_pos[0] + x > 14:
                return False
            # check for obstacles
            elif self.explored_map[current_pos[1] + y][current_pos[0] + x] == Cell.OBSTACLE:
                return False

        return True

    def is_pos_safe(self, pos, consider_unexplored=True):
        x = pos[0]
        y = pos[1]

        if x < 1 or x > 13 or y < 1 or y > 18:
            return False

        for col in range(x - 1, x + 2):
            for row in range(y - 1, y + 2):
                if self.explored_map[row][col] == Cell.OBSTACLE or (
                        consider_unexplored and self.explored_map[row][col] == Cell.UNEXPLORED):
                    return False

        return True

    def fastest_path_to_pos_to_check(self, pos_to_check):
        if len(pos_to_check) == 0:
            return False

        best_pos = min(pos_to_check.keys(), key=lambda pos_i: FastestPath.heuristic_function(self.robot.pos, pos_i))
        direction = pos_to_check[best_pos]

        fp = FastestPath(self.explored_map, self.robot.direction, self.robot.pos, best_pos)
        movements = fp.movements

        if movements is None:
            print("Can't go pos to check")
            return False

        for movement in movements:
            if self.is_limit_exceeded:
                return

            self.move(movement)

        num_rotate_right = (direction - self.robot.direction) % 4 if direction is not None else 0

        if num_rotate_right == 2:
            self.move(Movement.RIGHT)
            self.move(Movement.RIGHT)

        elif num_rotate_right == 1:
            self.move(Movement.RIGHT)

        elif num_rotate_right == 3:
            self.move(Movement.LEFT)

        return True

    def find_unexplored_to_check(self):
        pos_to_check = {}

        for r in range(20):
            for c in range(15):
                if self.explored_map[r][c] == Cell.UNEXPLORED:
                    for pos, direction in self.possible_cell_pos((c, r)):
                        pos_to_check[pos] = direction

        return pos_to_check

    def possible_cell_pos(self, goal):
        d = set()
        x, y = goal
        arr = [(0, -2), (-1, -2), (1, -2), (0, 2), (-1, 2), (1, 2), (2, 0), (2, -1), (2, 1), (-2, 0), (-2, 1), (-2, -1)]

        for i in arr:
            pos = (x + i[0], y + i[1])

            if self.is_pos_safe(pos):
                if pos[0] - x == 2:
                    direction = Direction.WEST
                elif pos[0] - x == -2:
                    direction = Direction.EAST
                elif pos[1] - y == 2:
                    direction = Direction.SOUTH
                elif pos[1] - y == -2:
                    direction = Direction.NORTH
                else:
                    raise ValueError

                d.add((pos, direction))

        return d

    def find_remaining_unexplored(self):
        pos_to_check = {}

        for r in range(20):
            for c in range(15):
                if self.explored_map[r][c] == Cell.UNEXPLORED:
                    for pos, direction in self.possible_remaining_unexplored((c, r)):
                        pos_to_check[pos] = direction

        return pos_to_check

    def possible_remaining_unexplored(self, goal):
        d = set()

        for direction in range(4):
            for sensor in self.robot.sensors:
                sensor_direction = (direction + sensor.direction - Direction.EAST) % 4
                reverse_sensor_direction = (sensor_direction + 2) % 4
                direction_vector = Direction.get_direction_vector(reverse_sensor_direction)

                for i in range(*sensor.get_range()):
                    sensor_pos = (goal[0] + i * direction_vector[0], goal[1] + i * direction_vector[1])

                    if sensor_pos[0] < 0 or sensor_pos[0] >= NUM_COLS or sensor_pos[1] < 0 or sensor_pos[
                        1] >= NUM_ROWS or self.explored_map[sensor_pos[1]][sensor_pos[0]] == Cell.OBSTACLE:
                        break

                    if direction == Direction.NORTH:
                        pos = sensor_pos[0] + sensor.pos[1], sensor_pos[1] - sensor.pos[0]
                    elif direction == Direction.EAST:
                        pos = sensor_pos[0] - sensor.pos[0], sensor_pos[1] - sensor.pos[1]
                    elif direction == Direction.SOUTH:
                        pos = sensor_pos[0] - sensor.pos[1], sensor_pos[1] + sensor.pos[0]
                    else:  # Direction.WEST
                        pos = sensor_pos[0] + sensor.pos[0], sensor_pos[1] + sensor.pos[1]

                    if self.is_pos_safe(pos):
                        d.add((pos, direction))

        return d

    def is_stuck_in_loop(self):
        if list(self.queue) == [Movement.FORWARD, Movement.RIGHT, Movement.FORWARD, Movement.RIGHT, Movement.FORWARD,
                                Movement.RIGHT]:
            return True
        return False

    def right_hug(self):
        time_right_hug = time.time()
        flag = True
        while True and flag:
            print("FLAG IS",flag)
            if self.is_limit_exceeded:
                break

            # print_map(self.explored_map, [self.robot.pos])
            if self.entered_goal and self.robot.pos == START_POS:
                break

            if self.robot.pos == GOAL_POS:
                self.entered_goal = True

            if self.is_stuck_in_loop():
                flag=self.move(Movement.RIGHT)
                if not flag:
                    break
                self.move(Movement.RIGHT)

            elif self.check_right():
                flag =self.move(Movement.RIGHT)
                if not flag:
                    break

            elif self.check_forward():
                flag =self.move(Movement.FORWARD)
                if not flag:
                    break

            elif self.check_left():
                flag =self.move(Movement.LEFT)
                if not flag:
                    break

            else:
                flag = self.move(Movement.LEFT)
                if not flag:
                    break
                self.move(Movement.LEFT)

        print("Time right hug: ", time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time_right_hug)))

    def explore_unexplored(self):
        print("Exploration: Exploring Unexplored Area")
        while True:
            if self.is_limit_exceeded:
                break

            unexplored_pos_to_check = self.find_unexplored_to_check()
            can_find = self.fastest_path_to_pos_to_check(unexplored_pos_to_check)
            if not can_find:
                break

        while True:
            if self.is_limit_exceeded:
                break

            unexplored_pos_to_check = self.find_remaining_unexplored()
            can_find = self.fastest_path_to_pos_to_check(unexplored_pos_to_check)
            if not can_find:
                break

    def fastest_path_to_start(self):
        print("Exploration: Starting fastest path to start")
        fp = FastestPath(self.explored_map, self.robot.direction, self.robot.pos, START_POS)
        movements = fp.movements
        if movements is None:
            print("Can't go back to start?")
            return

        for movement in movements:
            if not self.is_running:
                break
            print("Fastest Path, moving: ", movement)
            self.move(movement)

    def run_exploration(self):
        self.start_time = time.time()
        self.sense_and_repaint()
        self.right_hug()
        # self.explore_unexplored()  # might omit
        # self.fastest_path_to_start() # might omit

    def calibrate(self, sense):
        is_calibrated = False
        front_direction = self.robot.direction
        right_direction = (front_direction + 1) % 4
        front_direction_vector = Direction.get_direction_vector(front_direction)
        right_direction_vector = Direction.get_direction_vector(right_direction)

        # if self.steps_without_calibration >= MIN_STEPS_WITHOUT_CALIBRATION:
        # Check front
        can_calibrate_front = False
        for i in range(-1, 2):
            c = self.robot.pos[0] + 2 * front_direction_vector[0] + i * right_direction_vector[0]
            r = self.robot.pos[1] + 2 * front_direction_vector[1] + i * right_direction_vector[1]

            if c < 0 or c > NUM_COLS - 1 or r < 0 or r > NUM_ROWS - 1 or self.explored_map[r][c] == Cell.OBSTACLE:
                can_calibrate_front = True
                break

        if can_calibrate_front:
            self.on_calibrate(is_front=True)
            is_calibrated = True

        # Check right
        if self.steps_without_calibration >= MIN_STEPS_WITHOUT_CALIBRATION:
            can_calibrate_right = True
            for i in [-1, 1]:
                c = self.robot.pos[0] + i * front_direction_vector[0] + 2 * right_direction_vector[0]
                r = self.robot.pos[1] + i * front_direction_vector[1] + 2 * right_direction_vector[1]

                if 0 <= c < NUM_COLS and 0 <= r < NUM_ROWS and self.explored_map[r][c] != Cell.OBSTACLE:
                    can_calibrate_right = False

            if can_calibrate_right:
                self.on_calibrate(is_front=False)
                self.steps_without_calibration = 0
                is_calibrated = True

        if is_calibrated and sense:
            self.sense_and_repaint()

    def move(self, movement, sense=True):
        self.queue.append(movement)
        if len(self.queue) > 6:
            self.queue.popleft()
        if not isinstance(movement, Movement) or movement == Movement.FORWARD or movement == Movement.BACKWARD:
            self.prev_pos = self.robot.pos

        sensor_values = self.robot.move(movement)
        if sensor_values=="STOP":
            print("I ASKED TO STOP")
            return False
        if sense:
            self.sense_and_repaint(sensor_values)

        self.calibrate(sense)
        self.steps_without_calibration += 1
        return True

    def sense_and_repaint(self, sensor_values=None):
        if sensor_values is None:
            sensor_values = self.robot.sense()

        for i in range(len(sensor_values)):
            sensor_value = sensor_values[i]
            sensor = self.robot.sensors[i]
            direction_vector = Direction.get_direction_vector(sensor.get_current_direction(self.robot))
            current_sensor_pos = sensor.get_current_pos(self.robot)

            if sensor_value == -1:
                continue

            elif sensor_value is None:  # If sensor value is empty, means no obstacle in front
                for j in range(*sensor.get_range()):
                    pos_to_mark = (
                        current_sensor_pos[0] + j * direction_vector[0],
                        current_sensor_pos[1] + j * direction_vector[1])

                    if 0 <= pos_to_mark[0] <= NUM_COLS - 1 and 0 <= pos_to_mark[1] <= NUM_ROWS - 1:
                        self.explored_map[pos_to_mark[1]][pos_to_mark[0]] = Cell.FREE

            else:  # sensor value picked up something, obstacle in front
                for j in range(sensor.get_range()[0], min(sensor.get_range()[1], sensor_value + 1)):
                    pos_to_mark = (
                        current_sensor_pos[0] + j * direction_vector[0],
                        current_sensor_pos[1] + j * direction_vector[1])

                    if 0 <= pos_to_mark[0] <= NUM_COLS - 1 and 0 <= pos_to_mark[1] <= NUM_ROWS - 1:
                        self.explored_map[pos_to_mark[1]][
                            pos_to_mark[0]] = Cell.FREE if j != sensor_value else Cell.OBSTACLE

        for r in range(START_POS[1] - 1, START_POS[1] + 2):
            for c in range(START_POS[0] - 1, START_POS[0] + 2):
                self.explored_map[r][c] = Cell.FREE

        for r in range(GOAL_POS[1] - 1, GOAL_POS[1] + 2):
            for c in range(GOAL_POS[0] - 1, GOAL_POS[0] + 2):
                self.explored_map[r][c] = Cell.FREE

        self.on_update_map()


def main():
    with open("../maps/fastest_path_arena.txt", "r") as f:
        strs = f.read().split("\n")

    map_real = generate_map(*strs)
    bot = SimulatorBot(START_POS, Direction.EAST, lambda m: None)
    bot.map = map_real

    # exp = Exploration(bot, lambda: None)
    # exp.run_exploration()
    # print_map(exp.explored_map)
    print_map(map_real)


if __name__ == '__main__':
    main()
