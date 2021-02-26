from enum import IntEnum


class Cell(IntEnum):
	UNEXPLORED = 0
	FREE = 1
	OBSTACLE = 2


class Direction(IntEnum):
	NORTH = 0
	EAST = 1
	SOUTH = 2
	WEST = 3

	@staticmethod
	def get_direction_vector(direction):
		if direction == Direction.NORTH:
			return 0, 1
		elif direction == Direction.EAST:
			return 1, 0
		elif direction == Direction.SOUTH:
			return 0, -1
		else:  # Direction.WEST
			return -1, 0

	@staticmethod
	def convert_to_string(direction):
		if direction == Direction.NORTH:
			return "N"
		elif direction == Direction.EAST:
			return "E"
		elif direction == Direction.SOUTH:
			return "S"
		elif direction == Direction.WEST:
			return "W"
		else:
			raise ValueError

	@staticmethod
	def convert_from_string(direction_str):
		if direction_str == "N":
			return Direction.NORTH
		elif direction_str == "E":
			return Direction.EAST
		elif direction_str == "S":
			return Direction.SOUTH
		elif direction_str == "W":
			return Direction.WEST
		else:
			raise ValueError


class Movement(IntEnum):
	FORWARD = 0
	RIGHT = 1
	BACKWARD = 2
	LEFT = 3

	@staticmethod
	def convert_to_string(movement):
		if movement == Movement.FORWARD:
			return "F1"
		elif movement == Movement.RIGHT:
			return "R1"
		elif movement == Movement.BACKWARD:
			return "B1"
		else:  # Movement.LEFT
			return "L1"
