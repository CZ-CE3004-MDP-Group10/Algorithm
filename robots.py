import time

from constants import NUM_ROWS, NUM_COLS
from enums import Direction, Movement, Cell


class Robot:
	def __init__(self, pos, direction, on_move=None):
		"""
		Args:
			pos (tuple): Position of robot.
			direction (enums.Direction): Direction of robot.
			on_move (function): Callback function for when the robot moves.
		"""
		self.pos = pos
		self.direction = direction
		self.on_move = on_move if on_move is not None else lambda movement: None
		self.sensors = [
			Sensor(False, (1, 1), Direction.NORTH),
			Sensor(True, (1, 1), Direction.EAST),
			Sensor(True, (1, 0), Direction.EAST),
			Sensor(True, (1, -1), Direction.EAST),
			Sensor(True, (1, -1), Direction.SOUTH),
			Sensor(True, (-1, -1), Direction.SOUTH),
		]

	@property
	def speed(self):
		return None

	def move(self, movement, sense=False):
		if not isinstance(movement, Movement):
			if self.direction == Direction.NORTH:
				self.pos = (self.pos[0], self.pos[1] + movement)
			elif self.direction == Direction.EAST:
				self.pos = (self.pos[0] + movement, self.pos[1])
			elif self.direction == Direction.SOUTH:
				self.pos = (self.pos[0], self.pos[1] - movement)
			elif self.direction == Direction.WEST:
				self.pos = (self.pos[0] - movement, self.pos[1])

		elif movement == Movement.FORWARD:
			if self.direction == Direction.NORTH:
				self.pos = (self.pos[0], self.pos[1] + 1)
			elif self.direction == Direction.EAST:
				self.pos = (self.pos[0] + 1, self.pos[1])
			elif self.direction == Direction.SOUTH:
				self.pos = (self.pos[0], self.pos[1] - 1)
			elif self.direction == Direction.WEST:
				self.pos = (self.pos[0] - 1, self.pos[1])

		elif movement == Movement.BACKWARD:
			if self.direction == Direction.NORTH:
				self.pos = (self.pos[0], self.pos[1] - 1)
			elif self.direction == Direction.EAST:
				self.pos = (self.pos[0] - 1, self.pos[1])
			elif self.direction == Direction.SOUTH:
				self.pos = (self.pos[0], self.pos[1] + 1)
			elif self.direction == Direction.WEST:
				self.pos = (self.pos[0] + 1, self.pos[1])

		elif movement == Movement.RIGHT:
			self.direction = Direction((self.direction + 1) % 4)

		elif movement == Movement.LEFT:
			self.direction = Direction((self.direction - 1) % 4)

		return self.on_move(movement)
		
	def movefp(self, movement, sense=False):
		if not isinstance(movement, Movement):
			if self.direction == Direction.NORTH:
				self.pos = (self.pos[0], self.pos[1] + movement)
			elif self.direction == Direction.EAST:
				self.pos = (self.pos[0] + movement, self.pos[1])
			elif self.direction == Direction.SOUTH:
				self.pos = (self.pos[0], self.pos[1] - movement)
			elif self.direction == Direction.WEST:
				self.pos = (self.pos[0] - movement, self.pos[1])

		elif movement == Movement.FORWARD:
			if self.direction == Direction.NORTH:
				self.pos = (self.pos[0], self.pos[1] + 1)
			elif self.direction == Direction.EAST:
				self.pos = (self.pos[0] + 1, self.pos[1])
			elif self.direction == Direction.SOUTH:
				self.pos = (self.pos[0], self.pos[1] - 1)
			elif self.direction == Direction.WEST:
				self.pos = (self.pos[0] - 1, self.pos[1])

		elif movement == Movement.BACKWARD:
			if self.direction == Direction.NORTH:
				self.pos = (self.pos[0], self.pos[1] - 1)
			elif self.direction == Direction.EAST:
				self.pos = (self.pos[0] - 1, self.pos[1])
			elif self.direction == Direction.SOUTH:
				self.pos = (self.pos[0], self.pos[1] + 1)
			elif self.direction == Direction.WEST:
				self.pos = (self.pos[0] + 1, self.pos[1])

		elif movement == Movement.RIGHT:
			self.direction = Direction((self.direction + 1) % 4)

		elif movement == Movement.LEFT:
			self.direction = Direction((self.direction - 1) % 4)

	def sense(self):
		pass





class RealBot(Robot):
	def __init__(self, pos, direction, on_move, get_sensor_values):
		super(RealBot, self).__init__(pos, direction, on_move)
		self.get_sensor_values = get_sensor_values

	@property
	def speed(self):
		return 2

	def sense(self):
		print("Real sense")
		return self.get_sensor_values()


class SimulatorBot(Robot):
	def __init__(self, pos, direction, on_move=None, time_interval=0.2):
		super(SimulatorBot, self).__init__(pos, direction, on_move)
		self.map = []
		self.time_interval = time_interval

	@property
	def speed(self):
		return 1 / self.time_interval

	@speed.setter
	def speed(self, speed):
		self.time_interval = 1 / speed

	def move(self, movement, sense=False):
		# Wait to simulate robot
		time.sleep(self.time_interval)

		# Move virtual state
		super().move(movement, sense)

	def sense(self):
		sensor_values = []

		for sensor in self.sensors:
			direction_vector = Direction.get_direction_vector(sensor.get_current_direction(self))
			sensor_pos = sensor.get_current_pos(self)

			sensor_range = sensor.get_range()

			for i in range(1, sensor_range[1]):
				pos_to_check = (sensor_pos[0] + i * direction_vector[0], sensor_pos[1] + i * direction_vector[1])
				if pos_to_check[0] < 0 or pos_to_check[0] >= NUM_COLS or pos_to_check[1] < 0 or pos_to_check[1] >= NUM_ROWS or\
					self.map[pos_to_check[1]][pos_to_check[0]] == Cell.OBSTACLE:
					if i < sensor_range[0]:
						sensor_values.append(-1)
					else:
						sensor_values.append(i)
					break

			else:
				sensor_values.append(None)

		return sensor_values


class Sensor:
	# Inclusive at lower, exclusive at upper
	SR_RANGE = (1, 3)
	LR_RANGE = (1, 6)

	def __init__(self, is_short_range, pos, direction):
		self.is_short_range = is_short_range
		self.pos = pos
		self.direction = direction

	def get_range(self):
		if self.is_short_range:
			return Sensor.SR_RANGE
		else:
			return Sensor.LR_RANGE

	def get_current_direction(self, robot):
		return Direction((robot.direction + self.direction - Direction.EAST) % 4)

	def get_current_pos(self, robot):
		if robot.direction == Direction.NORTH:
			return robot.pos[0] - self.pos[1], robot.pos[1] + self.pos[0]
		elif robot.direction == Direction.EAST:
			return robot.pos[0] + self.pos[0], robot.pos[1] + self.pos[1]
		elif robot.direction == Direction.SOUTH:
			return robot.pos[0] + self.pos[1], robot.pos[1] - self.pos[0]
		else:  # Direction.WEST
			return robot.pos[0] - self.pos[0], robot.pos[1] - self.pos[1]
