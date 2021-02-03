from .exploration import Exploration
from enums import Cell, Direction
from map_descriptor import generate_map
from constants import START_POS, GOAL_POS, NUM_ROWS, NUM_COLS
from robots import SimulatorBot


class ShortImageRecExploration(Exploration):
    def __init__(self, robot, on_update_map=None, on_calibrate=None, explored_map=None, coverage_limit=None, time_limit=None, on_take_photo=None):
        super().__init__(robot, on_update_map=on_update_map, on_calibrate=on_calibrate, explored_map=explored_map, coverage_limit=coverage_limit, time_limit=time_limit)

        self.obstacles = {}

        if on_take_photo is None:
            self.on_take_photo = lambda obstacles, robot=None: None
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
                        self.obstacles[(pos[0] + i, pos[1] + 3)].remove(2)
                        obstacles.append((pos[0] + i, pos[1] + 3))
            if (pos[0], pos[1] + 2) in self.obstacles:
                if 2 in self.obstacles[(pos[0], pos[1] + 2)]:
                    self.obstacles[(pos[0], pos[1] + 2)].remove(2)
                    obstacles.append((pos[0], pos[1] + 2))
        elif direction == 1:
            for i in range(-1, 2):
                if (pos[0] + 3, pos[1] + i) in self.obstacles:
                    if 3 in self.obstacles[(pos[0] + 3, pos[1] + i)]:
                        self.obstacles[(pos[0] + 3, pos[1] + i)].remove(3)
                        obstacles.append((pos[0] + 3, pos[1] + i))
            if (pos[0] + 2, pos[1]) in self.obstacles:
                if 3 in self.obstacles[(pos[0] + 2, pos[1])]:
                    self.obstacles[(pos[0] + 2, pos[1])].remove(3)
                    obstacles.append((pos[0] + 2, pos[1]))

        elif direction == 2:
            for i in range(-1, 2):
                if (pos[0] + i, pos[1] - 3) in self.obstacles:
                    if 0 in self.obstacles[(pos[0] + i, pos[1] - 3)]:
                        self.obstacles[(pos[0] + i, pos[1] - 3)].remove(0)
                        obstacles.append((pos[0] + i, pos[1] - 3))
            if (pos[0], pos[1] - 2) in self.obstacles:
                if 0 in self.obstacles[(pos[0], pos[1] - 2)]:
                    self.obstacles[(pos[0], pos[1] - 2)].remove(0)
                    obstacles.append((pos[0], pos[1] - 2))

        elif direction == 3:
            for i in range(-1, 2):
                if (pos[0] - 3, pos[1] + i) in self.obstacles:
                    if 1 in self.obstacles[(pos[0] - 3, pos[1] + i)]:
                        self.obstacles[(pos[0] - 3, pos[1] + i)].remove(1)
                        obstacles.append((pos[0] - 3, pos[1] + i))
            if (pos[0] - 2, pos[1]) in self.obstacles:
                if 1 in self.obstacles[(pos[0] - 2, pos[1])]:
                    self.obstacles[(pos[0] - 2, pos[1])].remove(1)
                    obstacles.append((pos[0] - 2, pos[1]))

        return obstacles

    def snap_obstacle_side(self):
        print('here')
        direction = self.robot.direction
        pos = self.robot.pos

        # if right side got obstacles with sides never see before, take photo
        right = (direction + 1) % 4
        obstacles = self.check_obstacle_side(pos, right)
        if len(obstacles) != 0:
            self.on_take_photo(obstacles)
            print('right take photo')

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

    def move(self, movement, sense=True):
        super(ShortImageRecExploration, self).move(movement, sense)
        self.snap_obstacle_side()


def main():
    with open("../maps/sample_arena5.txt", "r") as f:
        strs = f.read().split("\n")

    map_real = generate_map(*strs)
    bot = SimulatorBot(START_POS, Direction.EAST, lambda m: None)
    bot.map = map_real
    exp = ShortImageRecExploration(bot, lambda: None)
    exp.run_exploration()
    print(exp.obstacles)


if __name__ == '__main__':
    main()
