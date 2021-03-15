from .exploration import Exploration
from enums import Cell, Direction, Movement
from map_descriptor import generate_map
from constants import START_POS, GOAL_POS, NUM_ROWS, NUM_COLS
from robots import SimulatorBot
import time

class CompleteImageRecExploration(Exploration):
    def __init__(self, robot, on_update_map=None, on_calibrate=None, explored_map=None, coverage_limit=None, time_limit=None, on_take_photo=None):
        super().__init__(robot, on_update_map=on_update_map, on_calibrate=on_calibrate, explored_map=explored_map, coverage_limit=coverage_limit, time_limit=time_limit)

        self.obstacles = {}

        if on_take_photo is None:
            self.on_take_photo = lambda obstacles: None
        else:
            self.on_take_photo = on_take_photo

    def remove_obstacle_side(self, pos):
        for i in range(1, 4):
            if (pos[0] + i, pos[1]) in self.obstacles:
                if 1 in self.obstacles[pos]:
                    self.obstacles[pos].remove(1)
                if 3 in self.obstacles[(pos[0] + i, pos[1])]:
                    self.obstacles[(pos[0] + i, pos[1])].remove(3)
            if pos[0] + i > 14 and 1 in self.obstacles[pos]:
                self.obstacles[pos].remove(1)
            if (pos[0] - i, pos[1]) in self.obstacles:
                if 3 in self.obstacles[pos]:
                    self.obstacles[pos].remove(3)
                if 1 in self.obstacles[(pos[0] - i, pos[1])]:
                    self.obstacles[(pos[0] - i, pos[1])].remove(1)
            if pos[0] - i < 0 and 3 in self.obstacles[pos]:
                self.obstacles[pos].remove(3)
            if (pos[0], pos[1] + i) in self.obstacles:
                if 0 in self.obstacles[pos]:
                    self.obstacles[pos].remove(0)
                if 2 in self.obstacles[(pos[0], pos[1] + i)]:
                    self.obstacles[(pos[0], pos[1] + i)].remove(2)
            if pos[1] + i > 19 and 0 in self.obstacles[pos]:
                self.obstacles[pos].remove(0)
            if (pos[0], pos[1] - i) in self.obstacles:
                if 2 in self.obstacles[pos]:
                    self.obstacles[pos].remove(2)
                if 0 in self.obstacles[(pos[0], pos[1] - i)]:
                    self.obstacles[(pos[0], pos[1] - i)].remove(0)
            if pos[1] - i < 0 and 2 in self.obstacles[pos]:
                self.obstacles[pos].remove(2)

    def check_obstacle_side(self, pos, direction):
        obstacles = []
        if direction == 0:
            for i in range(-1, 2):
                if (pos[0] + i, pos[1] + 3) in self.obstacles:
                    if 2 in self.obstacles[(pos[0] + i, pos[1] + 3)]:
                        obstacles.append((pos[0] + i, pos[1] + 3))
            if (pos[0], pos[1] + 2) in self.obstacles:
                if 2 in self.obstacles[(pos[0], pos[1] + 2)]:
                    obstacles.append((pos[0], pos[1] + 2))

        elif direction == 1:
            for i in range(-1, 2):
                if (pos[0] + 3, pos[1] + i) in self.obstacles:
                    if 3 in self.obstacles[(pos[0] + 3, pos[1] + i)]:
                        obstacles.append((pos[0] + 3, pos[1] + i))
            if (pos[0] + 2, pos[1]) in self.obstacles:
                if 3 in self.obstacles[(pos[0] + 2, pos[1])]:
                    obstacles.append((pos[0] + 2, pos[1]))

        elif direction == 2:
            for i in range(-1, 2):
                if (pos[0] + i, pos[1] - 3) in self.obstacles:
                    if 0 in self.obstacles[(pos[0] + i, pos[1] - 3)]:
                        obstacles.append((pos[0] + i, pos[1] - 3))
            if (pos[0], pos[1] - 2) in self.obstacles:
                if 0 in self.obstacles[(pos[0], pos[1] - 2)]:
                    obstacles.append((pos[0], pos[1] - 2))

        elif direction == 3:
            for i in range(-1, 2):
                if (pos[0] - 3, pos[1] + i) in self.obstacles:
                    if 1 in self.obstacles[(pos[0] - 3, pos[1] + i)]:
                        obstacles.append((pos[0] - 3, pos[1] + i))
            if (pos[0] - 2, pos[1]) in self.obstacles:
                if 1 in self.obstacles[(pos[0] - 2, pos[1])]:
                    obstacles.append((pos[0] - 2, pos[1]))

        return obstacles

    def check_if_contains_corners(self, obstacles, direction):
        obstacle_direction = (direction + 2) % 4
        direction_vector = Direction.get_direction_vector(obstacle_direction)
        for obstacle in obstacles:
            pos = (obstacle[0] + 2 * direction_vector[0], obstacle[1] + 2 * direction_vector[1])

            if not self.is_pos_safe(pos, consider_unexplored=False):
                print(pos, obstacle)
                return True

        return False

    def snap_obstacle_side(self):
        direction = self.robot.direction
        pos = self.robot.pos

        # if right side got obstacles with sides never see before, take photo
        right = (direction + 1) % 4
        obstacles = self.check_obstacle_side(pos, right)
        if len(obstacles) != 0:
            for i in obstacles:
                self.obstacles[i].remove((right + 2) % 4)
            self.on_take_photo(obstacles, self.robot)
            print('right take photo')

        # if front got obstacles with sides never see before, turn and take photo
        obstacles = self.check_obstacle_side(pos, direction)
        has_front = False
        if len(obstacles) != 0 and self.check_if_contains_corners(obstacles, direction):
            for i in obstacles:
                self.obstacles[i].remove((direction + 2) % 4)
            self.move(Movement.LEFT)
            self.on_take_photo(obstacles, self.robot)
            print('front take photo')
            has_front = True

        # if left side got obstacles with sides never see before, turn and take photo
        left = (direction - 1) % 4
        obstacles = self.check_obstacle_side(pos, left)
        has_left = False
        if len(obstacles) != 0 and self.check_if_contains_corners(obstacles, left):
            for i in obstacles:
                self.obstacles[i].remove((left + 2) % 4)

            if not has_front:
                self.move(Movement.LEFT)
            self.move(Movement.LEFT)
            self.on_take_photo(obstacles, self.robot)
            print('left take photo')
            has_left = True
        elif has_front:
            self.move(Movement.RIGHT)

        # if back got obstacles....
        back = (direction + 2) % 4
        obstacles = self.check_obstacle_side(pos, back)
        if len(obstacles) != 0 and self.check_if_contains_corners(obstacles, back):
            for i in obstacles:
                self.obstacles[i].remove((back + 2) % 4)

            if not has_left:
                self.move(Movement.RIGHT)
            else:
                self.move(Movement.LEFT)
            print('back take photo')
            self.on_take_photo(obstacles, self.robot)
            self.move(Movement.LEFT)

        elif has_left:
            self.move(Movement.RIGHT)
            self.move(Movement.RIGHT)

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

            elif sensor_value is None:
                for j in range(*sensor.get_range()):
                    pos_to_mark = (current_sensor_pos[0] + j * direction_vector[0], current_sensor_pos[1] + j * direction_vector[1])

                    if 0 <= pos_to_mark[0] <= NUM_COLS - 1 and 0 <= pos_to_mark[1] <= NUM_ROWS - 1:
                        self.explored_map[pos_to_mark[1]][pos_to_mark[0]] = Cell.FREE

            else:
                for j in range(sensor.get_range()[0], min(sensor.get_range()[1], sensor_value + 1)):
                    pos_to_mark = (current_sensor_pos[0] + j * direction_vector[0], current_sensor_pos[1] + j * direction_vector[1])

                    if 0 <= pos_to_mark[0] <= NUM_COLS - 1 and 0 <= pos_to_mark[1] <= NUM_ROWS - 1:
                        if j != sensor_value:
                            self.explored_map[pos_to_mark[1]][pos_to_mark[0]] = Cell.FREE
                        else:
                            self.explored_map[pos_to_mark[1]][pos_to_mark[0]] = Cell.OBSTACLE
                            if pos_to_mark not in self.obstacles:
                                self.obstacles[pos_to_mark] = {0, 1, 2, 3}
                                self.remove_obstacle_side(pos_to_mark)

        for r in range(START_POS[1] - 1, START_POS[1] + 2):
            for c in range(START_POS[0] - 1, START_POS[0] + 2):
                self.explored_map[r][c] = Cell.FREE

        for r in range(GOAL_POS[1] - 1, GOAL_POS[1] + 2):
            for c in range(GOAL_POS[0] - 1, GOAL_POS[0] + 2):
                self.explored_map[r][c] = Cell.FREE

        self.on_update_map()

    def hug_middle_obstacles(self):
        print("HUG MIDDLE OBSTACLES")
        while True:
            if self.is_limit_exceeded:
                break

            obstacles_to_hug = self.find_obstacles_to_hug()
            can_find = self.fastest_path_to_pos_to_check(obstacles_to_hug)

            if can_find:
                self.right_hug_obstacle(self.robot.pos)
            else:
                break

    def right_hug_obstacle(self, initial_pos):
        while True:
            if self.is_limit_exceeded:
                break

            if self.check_right():
                self.move(Movement.RIGHT)

            elif self.check_forward():
                self.move(Movement.FORWARD)

            elif self.check_left():
                self.move(Movement.LEFT)

            else:
                self.move(Movement.LEFT)
                self.move(Movement.LEFT)

            if self.robot.pos == initial_pos:
                break

    def explore_unseen(self):
        print("EXPLORE UNSEEN")
        while True:
            if self.is_limit_exceeded:
                break

            unseen_pos_to_check = self.find_unseen_to_check()
            can_find = self.fastest_path_to_pos_to_check(unseen_pos_to_check)

            if not can_find:
                break

    def find_obstacles_to_hug(self):
        pos_to_check = {}

        for obstacle_pos in self.obstacles:
            for obstacle_direction in self.obstacles[obstacle_pos]:
                for pos, direction in self.possible_hug_pos(obstacle_pos, obstacle_direction):
                    pos_to_check[pos] = direction

        return pos_to_check

    def possible_hug_pos(self, goal, direction):
        d = set()
        x, y = goal
        correct_direction = (direction + 1) % 4

        if direction == Direction.NORTH:
            offset = (0, 2)
        elif direction == Direction.EAST:
            offset = (2, 0)
        elif direction == Direction.SOUTH:
            offset = (0, -2)
        elif direction == Direction.WEST:
            offset = (-2, 0)
        else:
            print('GGWP')
            raise ValueError

        pos = (x + offset[0], y + offset[1])

        if self.is_pos_safe(pos):
            d.add((pos, correct_direction))

        return d

    def find_unseen_to_check(self):
        pos_to_check = {}

        for obstacle_pos in self.obstacles:
            for obstacle_direction in self.obstacles[obstacle_pos]:
                for pos, direction in self.possible_photo_pos(obstacle_pos, obstacle_direction):
                    pos_to_check[pos] = direction

        return pos_to_check


    def possible_photo_pos(self, goal, direction):
        d = set()
        x, y = goal
        correct_direction = (direction + 1) % 4

        if direction == Direction.NORTH:
            arr = [(0, 2), (-1, 3), (0, 3), (1, 3)]
        elif direction == Direction.EAST:
            arr = [(2, 0), (3, -1), (3, 0), (3, 1)]
        elif direction == Direction.SOUTH:
            arr = [(0, -2), (-1, -3), (0, -3), (1, -3)]
        elif direction == Direction.WEST:
            arr = [(-2, 0), (-3, -1), (-3, 0), (-3, 1)]
        else:
            print('GGWP')
            raise ValueError

        for i in arr:
            pos = (x + i[0], y + i[1])

            if self.is_pos_safe(pos):
                d.add((pos, correct_direction))

        return d

    def run_exploration(self):
        self.start_time = time.time()
        self.sense_and_repaint()
        self.right_hug()
        self.hug_middle_obstacles()
        #self.explore_unexplored()
        #self.explore_unseen()
        #self.fastest_path_to_start()

    def move(self, movement, sense=True):
        super(CompleteImageRecExploration, self).move(movement, sense)
        self.snap_obstacle_side()


def main():
    with open("../maps/sample_arena5.txt", "r") as f:
        strs = f.read().split("\n")

    map_real = generate_map(*strs)
    bot = SimulatorBot(START_POS, Direction.EAST, lambda m: None)
    bot.map = map_real
    exp = CompleteImageRecExploration(bot, lambda: None)
    exp.run_exploration()
    print(exp.obstacles)


if __name__ == '__main__':
    main()