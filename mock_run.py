from rpi import RPi
from fastest_path.calibrate_fastest_path import CalibrateFastestPath
from exploration.short_image_rec_exploration import ShortImageRecExploration
from exploration.complete_image_rec_exploration import CompleteImageRecExploration
from threading import Thread
from constants import START_POS, NUM_ROWS, NUM_COLS
from robots import RealBot
from enums import Direction, Cell, Movement
from map_descriptor import generate_map_descriptor
from gui import GUI
from utils import generate_unexplored_map
import time

# Set to True for complete image recognition exploration
USE_COMPLETE_IMAGE_REC_EXPLORATION = True


class MockRun:
    def __init__(self):
        self.rpi = RPi(on_quit=self.on_quit)
        self.robot = RealBot(
            pos=START_POS,
            direction=Direction.EAST,
            on_move=self.on_move,
            get_sensor_values=self.rpi.receive_sensor_values,
        )
        self.exp = None
        self.is_running = False
        self.explored_map = generate_unexplored_map()
        self.waypoint = None

        self.gui = GUI(self.explored_map, self.robot)

    def start(self):
        self.rpi.open_connection()
        self.rpi.ping()
        Thread(target=self.connect_to_rpi).start()
        self.rpi.receive_endlessly()

    def connect_to_rpi(self):
        while True:
            msg_type, msg = self.rpi.receive_msg_with_type()

            time.sleep(10)
            # Fastest Path
            # if msg_type == RPi.FASTEST_PATH_MSG:
            self.is_running = True

            self.robot.pos = START_POS
            self.update_gui()

            cfp = CalibrateFastestPath(
                robot=self.robot,
                on_calibrate=self.rpi.calibrate,
                explored_map=self.explored_map,
                waypoint=self.waypoint
            )

            # Run fastest path
            cfp.run_fastest_path()

            self.rpi.send(RPi.FASTEST_PATH_MSG)
            print("FASTEST PATH COMPLETE!")

            self.is_running = False

    def display_gui(self):
        self.gui.start()

    def update_gui(self):
        self.gui.update_canvas()

    def on_move(self, movement):
        sensor_values = self.rpi.send_movement(movement, self.robot)
        self.update_gui()
        return sensor_values

    def on_update(self):
        self.rpi.send_map(self.explored_map)
        self.update_gui()

    def calibrate(self):
        if self.robot.direction == Direction.NORTH:
            # Calibrate facing south wall
            self.robot.move(Movement.LEFT)
            self.robot.move(Movement.LEFT)
            self.rpi.calibrate(is_front=True)
            self.rpi.calibrate(is_front=False)

            # Calibrate facing west wall
            self.robot.move(Movement.RIGHT)
            self.rpi.calibrate(is_front=True)

            # Turn back
            self.robot.move(Movement.RIGHT)

        elif self.robot.direction == Direction.EAST:
            # Calibrate facing south wall
            self.robot.move(Movement.RIGHT)
            self.rpi.calibrate(is_front=True)
            self.rpi.calibrate(is_front=False)

            # Turn back
            self.robot.move(Movement.LEFT)

            # Calibrate facing east
            self.rpi.calibrate(is_front=False)

        elif self.robot.direction == Direction.SOUTH:
            # Calibrate facing south wall
            self.rpi.calibrate(is_front=True)
            self.rpi.calibrate(is_front=False)

        elif self.robot.direction == Direction.WEST:
            # Calibrate facing south wall
            self.robot.move(Movement.LEFT)
            self.rpi.calibrate(is_front=True)
            self.rpi.calibrate(is_front=False)

            # Turn back
            self.robot.move(Movement.RIGHT)

            # Calibrate facing west wall
            self.rpi.calibrate(is_front=True)

    def on_quit(self):
        if self.exp:
            self.exp.is_running = False
        self.is_running = False


if __name__ == '__main__':
    mr = MockRun()
    Thread(target=mr.start).start()
    #mr.display_gui()
