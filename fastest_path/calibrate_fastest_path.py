from enums import Cell, Direction
from utils import print_map, generate_unexplored_map
from map_descriptor import generate_map
from constants import START_POS, GOAL_POS, NUM_ROWS, NUM_COLS
from robots import SimulatorBot
from .fastest_path import FastestPath
import time

COMBINED_MOVEMENT = True
MIN_STEPS_WITHOUT_CALIBRATION = 1000

class CalibrateFastestPath:
	def __init__(self, robot, on_calibrate=None, explored_map=None, waypoint=None):
		"""
		Args:
			robot (robots.Robot): Robot object to explore the map.
			on_update_map (function): Function called whenever the map is updated.
			explored_map (list): 2D list of Cell objects representing the map.
			coverage_limit (int): Coverage limit as a % (e.g. 100 means 100%).
			time_limit (int): Time limit in seconds.
		"""
		self.robot = robot
		self.prev_pos = None
		self.explored_map = explored_map if explored_map is not None else generate_unexplored_map()
		self.steps_without_calibration = 0
		self.obstacles = {}
		self.waypoint = waypoint

		if on_calibrate is None:
			self.on_calibrate = lambda is_front: None
		else:
			self.on_calibrate = on_calibrate

	def run_fastest_path(self):
		fp = FastestPath(self.explored_map, self.robot.direction, START_POS, GOAL_POS, self.waypoint)
		#movements = fp.combined_movements() if COMBINED_MOVEMENT else fp.movements
		movements = fp.custom_combined_movements() if COMBINED_MOVEMENT else fp.movements
		print(movements)

		return movements



	def calibrate(self):
		front_direction = self.robot.direction
		right_direction = (front_direction + 1) % 4
		front_direction_vector = Direction.get_direction_vector(front_direction)
		right_direction_vector = Direction.get_direction_vector(right_direction)

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


def main():
	with open("../maps/sample_arena1.txt", "r") as f:
		strs = f.read().split("\n")

	map_real = generate_map(*strs)
	bot = SimulatorBot(START_POS, Direction.EAST, lambda m: None)
	bot.map = map_real

	exp = CalibrateFastestPath(bot, lambda: None)
	exp.run_fastest_path()
	print_map(exp.explored_map)
	print_map(map_real)


if __name__ == '__main__':
	main()
