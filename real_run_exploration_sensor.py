from exploration.exploration import Exploration
from rpi import RPi
from fastest_path.calibrate_fastest_path import CalibrateFastestPath
from exploration.short_image_rec_exploration import ShortImageRecExploration
from exploration.complete_image_rec_exploration import CompleteImageRecExploration
from threading import Thread
from constants import START_POS, NUM_ROWS, NUM_COLS
from robots import RealBot
from enums import Direction, Cell, Movement
from map_descriptor import generate_map_descriptor
from map_descriptor import generate_map
from gui import GUI
from utils import generate_unexplored_map
import time
import re

# Set to True for complete image recognition exploration
USE_COMPLETE_IMAGE_REC_EXPLORATION = True


class RealRun:
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
        self.explored_map = generate_unexplored_map()  # Empty map to be explored
        self.waypoint = None
        self.gui = GUI(self.explored_map, self.robot)

    def start(self):
        self.rpi.open_connection()

        Thread(target=self.connect_to_rpi).start()
        self.rpi.receive_endlessly()

    def connect_to_rpi(self):

        msg, msg_type = self.rpi.receive_msg_with_type()

        msg_type = RPi.EXPLORE_MSG
        # Exploration
        if msg_type == RPi.EXPLORE_MSG:
            self.is_running = True
            self.rpi.set_speed(is_high=True)
            self.explored_map = generate_unexplored_map()
            self.gui.map = self.explored_map
            self.on_update()

            self.exp = CompleteImageRecExploration(
                robot=self.robot,
                on_update_map=self.on_update,
                on_calibrate=self.rpi.calibrate,
                on_take_photo=self.rpi.take_photo,
                explored_map=self.explored_map,
                time_limit=320
            )

            c, r = self.robot.pos
            for i in range(max(0, r - 1), min(NUM_ROWS, r + 2)):
                for j in range(max(0, c - 1), min(NUM_COLS, c + 2)):
                    self.exp.explored_map[i][j] = Cell.FREE

            self.update_gui()

            # Run exploration
            time.sleep(0.5)
            self.rpi.send("ARD|x")
            time.sleep(0.5)
            self.exp.run_exploration()


            # Prepare robot position for fastest path
            """
                if self.robot.pos == START_POS:
                    if self.robot.direction == Direction.SOUTH:
                        self.robot.move(Movement.LEFT)
                    elif self.robot.direction == Direction.WEST:
                        self.robot.move(Movement.RIGHT)

                    self.calibrate()
                """
            self.is_running = False
            self.rpi.send("CV|Q")
            self.rpi.send(RPi.EXPLORE_MSG)

        elif msg_type == "S":
            self.rpi.send("ARD|S")

    def display_gui(self):
        self.gui.start()

    def update_gui(self):
        self.gui.update_canvas()

    def on_move(self, movement):
        sensor_values = self.rpi.send_movement(movement, self.robot)
        self.update_gui()
        return sensor_values

    def on_update(self):
        self.update_gui()

       #self.rpi.send_obstacle_map(self.explored_map)
        #time.sleep(0.1)

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
    rr = RealRun()
    Thread(target=rr.start).start()
    rr.display_gui()
