import tkinter as tk
from tkinter import *
from tkinter import ttk
from tkinter.font import Font
from threading import Thread

from constants import NUM_ROWS, NUM_COLS, START_POS, GOAL_POS
from enums import Cell, Direction
from robots import SimulatorBot
from map_descriptor import generate_map, generate_map_descriptor
from fastest_path.fastest_path import FastestPath
from exploration.exploration import Exploration
from exploration.complete_image_rec_exploration import CompleteImageRecExploration


class GUI:
    # Size
    CELL_SIZE = 25  # 40

    # Colours
    BLACK = "#2E3134"
    LIGHT_GREY = "#D4DFE7"
    DARK_GREY = "#737880"
    WHITE = "#FFFFFF"
    RED = "#CB7373"
    BLUE = "#589DCE"
    YELLOW = "#DABF56"

    # TAGS
    MAP_TAG_0 = "map0"
    MAP_TAG_1 = "map2"
    ROBOT_TAG = "rob1t"

    def __init__(self, explored_map, robot):
        self.map_tag = True
        self.root = None
        self.canvas = None
        self.map = explored_map
        self.robot = robot
        self.waypoint = None

    def start(self):
        self.root = tk.Tk()
        self.root.configure(bg='lightblue')
        self.root.title("Algorithm Simulator")
        widget = Label(self.root,bg='lightblue')
        widget.config(text='Algo Simulator!') 
        widget.config(font=("Courier", 44))
        widget.pack(side=TOP, expand=YES, fill=BOTH)
        self.display_side_panel()
        self.display_canvas()

        self.root.mainloop()

    def display_side_panel(self):
        pass

    def display_canvas(self):
        self.canvas = tk.Canvas(self.root, height=NUM_ROWS * GUI.CELL_SIZE, width=NUM_COLS * GUI.CELL_SIZE, bg='white')
        self.canvas.pack(fill=tk.BOTH, expand=True, side=tk.LEFT, padx=0, pady=5)

        # self.canvas.pack(fill=tk.BOTH, expand=True, side=tk.LEFT, padx=20, pady=20)
        self.update_map()
        self.update_robot()

    def update_map(self):
        self.canvas.delete(GUI.MAP_TAG_1 if self.map_tag else GUI.MAP_TAG_0)

        for r in range(NUM_ROWS):
            for c in range(NUM_COLS):
                cell = self.map[r][c]
                x = c * GUI.CELL_SIZE
                y = (NUM_ROWS - 1 - r) * GUI.CELL_SIZE

                if c in range(START_POS[0] - 1, START_POS[0] + 2) and r in range(START_POS[1] - 1, START_POS[1] + 2):
                    color = GUI.BLUE
                elif c in range(GOAL_POS[0] - 1, GOAL_POS[0] + 2) and r in range(GOAL_POS[1] - 1, GOAL_POS[1] + 2):
                    color = GUI.RED
                elif (c, r) == self.waypoint:
                    color = GUI.YELLOW
                elif cell == Cell.UNEXPLORED:
                    color = GUI.DARK_GREY
                elif cell == Cell.FREE:
                    color = GUI.LIGHT_GREY
                else:  # Obstacle
                    color = GUI.BLACK

                self.canvas.create_rectangle(
                    x,
                    y,
                    x + GUI.CELL_SIZE,
                    y + GUI.CELL_SIZE,
                    width=6,
                    outline=GUI.WHITE,
                    fill=color,
                    tag=GUI.MAP_TAG_0 if self.map_tag else GUI.MAP_TAG_1,
                )

        self.map_tag = not self.map_tag

    def update_robot(self):
        self.canvas.delete(GUI.ROBOT_TAG)

        x = (self.robot.pos[0] - 0.5) * GUI.CELL_SIZE
        y = (NUM_ROWS - 1.5 - self.robot.pos[1]) * GUI.CELL_SIZE
        self.canvas.create_oval(
            x,
            y,
            x + 2 * GUI.CELL_SIZE,
            y + 2 * GUI.CELL_SIZE,
            width=0,
            fill=GUI.BLACK,
            tag=GUI.ROBOT_TAG,
        )

        if self.robot.direction == Direction.NORTH:
            self.canvas.create_rectangle(
                x + GUI.CELL_SIZE - 4,
                y + 10,
                x + GUI.CELL_SIZE + 4,
                y + 26,
                width=0, fill=GUI.WHITE, tag=GUI.ROBOT_TAG
            )

        elif self.robot.direction == Direction.EAST:
            self.canvas.create_rectangle(
                x + 2 * GUI.CELL_SIZE - 26,
                y + GUI.CELL_SIZE - 4,
                x + 2 * GUI.CELL_SIZE - 10,
                y + GUI.CELL_SIZE + 4,
                width=0, fill=GUI.WHITE, tag=GUI.ROBOT_TAG
            )

        elif self.robot.direction == Direction.SOUTH:
            self.canvas.create_rectangle(
                x + GUI.CELL_SIZE - 4,
                y + 2 * GUI.CELL_SIZE - 26,
                x + GUI.CELL_SIZE + 4,
                y + 2 * GUI.CELL_SIZE - 10,
                width=0, fill=GUI.WHITE, tag=GUI.ROBOT_TAG
            )

        elif self.robot.direction == Direction.WEST:
            self.canvas.create_rectangle(
                x + 10,
                y + GUI.CELL_SIZE - 4,
                x + 26,
                y + GUI.CELL_SIZE + 4,
                width=0, fill=GUI.WHITE, tag=GUI.ROBOT_TAG
            )

    def update_canvas(self):
        self.update_map()
        self.update_robot()


class SimulatorGUI(GUI):
    # MAP OPTIONS
    MAP_OPTIONS = {
        "Sample Arena 1": "maps/sample_arena1.txt",
        "Sample Arena 2": "maps/sample_arena2.txt",
        "Sample Arena 3": "maps/sample_arena3.txt",
        "Sample Arena 4": "maps/sample_arena4.txt",
        "Sample Arena 5": "maps/sample_arena5.txt",
        "Custom": None,
    }

    # SPEED
    MIN_SPEED = 1
    MAX_SPEED = 20

    def __init__(self):
        with open("maps/fastest_path_arena.txt", "r") as f:
            strs = f.read().split("\n")

        self.selected_map = generate_map(*strs)

        super(SimulatorGUI, self).__init__(
            self.selected_map.copy(),
            SimulatorBot(START_POS, Direction.EAST, on_move=lambda movement=None: self.update_robot())
        )

        self.current_thread = None
        self.error_frame = None
        self.exp = None

        # Variables
        self.exploration_coverage = None
        self.exploration_time = None
        self.is_running = False

        # Inputs
        self.selected_map_str = None
        self.mdf_input = None
        self.with_image_rec = None
        self.coverage_limit_input = None
        self.time_limit_input = None
        self.has_waypoint_input = None
        self.waypoint_x_input = None
        self.waypoint_y_input = None
        self.robot_speed = None

    def display_side_panel(self):

        side_panel = tk.Frame(self.root)
        side_panel.configure(bg='lightblue')
        left_sidepanel = tk.Frame(side_panel)
        left_sidepanel.configure(bg='lightblue')
        left_sidepanel.pack(side=LEFT, padx=(18, 10))
        right_sidepanel = tk.Frame(side_panel)
        right_sidepanel.configure(bg='lightblue')
        right_sidepanel.pack(side=RIGHT, padx=(10, 18))

        side_panel.pack(side=tk.LEFT, padx=0, pady=0)

        # Error Frame
        #self.error_frame = tk.Frame(left_sidepanel)
        #self.error_frame.pack(fill=tk.X, pady=0)
        #self.error_frame.configure(bg='lightblue')

        # Algorithms Frame
        algorithms_frame = tk.Frame(left_sidepanel,highlightbackground='black',highlightcolor="black", highlightthickness=1)
        algorithms_frame.pack(fill=tk.X,side=tk.TOP)

        self.create_heading(algorithms_frame, "Algorithms").pack()
        self.create_button(algorithms_frame, "Exploration", lambda: self.execute_thread(self.exploration)) \
            .pack(fill=tk.X)
        self.create_button(algorithms_frame, "Fastest Path", lambda: self.execute_thread(self.fastest_path)) \
            .pack(fill=tk.X)

        self.create_button(algorithms_frame, "Stop", lambda: self.stop()) \
            .pack(fill=tk.X)
        self.create_button(algorithms_frame, "Reset", lambda: self.execute_thread(self.reset)) \
            .pack(fill=tk.X)

        # Map Select Frame
        map_select_frame = tk.Frame(left_sidepanel,highlightbackground='black',highlightcolor="black", highlightthickness=1)

        # map_select_frame = tk.Frame(side_panel)
        map_select_frame.pack(fill=tk.X, pady=35)

        self.create_heading(map_select_frame, "Select Map").pack(fill=tk.X)

        map_option_strs = list(SimulatorGUI.MAP_OPTIONS.keys())
        self.selected_map_str = tk.StringVar()
        self.selected_map_str.set(map_option_strs[0])
        tk.OptionMenu(map_select_frame, self.selected_map_str, *map_option_strs).pack()

        tk.Label(map_select_frame, text="If custom, please specify MDF strings (separated by a comma)").pack()
        self.mdf_input = tk.Entry(map_select_frame)
        self.mdf_input.pack(fill=tk.X)

        self.create_button(map_select_frame, "Load Map", lambda: self.execute_thread(self.load_map)) \
            .pack(fill=tk.X)

        # Exploration Frame
        exploration_frame = tk.Frame(right_sidepanel,highlightbackground='black',highlightcolor="black", highlightthickness=1)
        exploration_frame.pack(fill=tk.X, pady=(0,10))

        self.create_heading(exploration_frame, "Exploration").pack(fill=tk.X)

        self.with_image_rec = tk.IntVar()
        tk.Checkbutton(exploration_frame, text="Use Image Recognition Algorithm", variable=self.with_image_rec).pack()

        self.exploration_coverage = tk.StringVar()
        self.exploration_coverage.set("Coverage: 0%")
        tk.Label(exploration_frame, textvariable=self.exploration_coverage).pack()

        self.exploration_time = tk.StringVar()
        self.exploration_time.set("Time: 00:00")
        tk.Label(exploration_frame, textvariable=self.exploration_time).pack()

        tk.Label(exploration_frame, text="Coverage Limit").pack()
        self.coverage_limit_input = tk.IntVar()
        self.coverage_limit_input.set(360)
        tk.Spinbox(exploration_frame, from_=0, to=100, text=self.coverage_limit_input).pack(fill=tk.X)

        tk.Label(exploration_frame, text="Time Limit").pack()
        self.time_limit_input = tk.IntVar()
        self.time_limit_input.set(360)
        tk.Spinbox(exploration_frame, from_=0, to=360, textvariable=self.time_limit_input).pack(fill=tk.X)

        # Waypoint Frame
        waypoint_frame = tk.Frame(right_sidepanel,highlightbackground='black',highlightcolor="black", highlightthickness=1)
        waypoint_frame.pack(fill=tk.X, pady=10)

        self.create_heading(waypoint_frame, "Waypoint").pack()

        self.has_waypoint_input = tk.IntVar()
        tk.Checkbutton(waypoint_frame, text="Use waypoint", variable=self.has_waypoint_input).pack()

        tk.Label(waypoint_frame, text="X").pack(side=tk.LEFT,padx=(35,0))
        self.waypoint_x_input = tk.IntVar()
        self.waypoint_x_input.set(GOAL_POS[0])
        tk.Spinbox(waypoint_frame, from_=0, to=NUM_COLS - 1, textvariable=self.waypoint_x_input) \
            .pack(fill=tk.X, side=tk.LEFT,padx=(0,20))

        tk.Label(waypoint_frame, text="Y").pack(side=tk.LEFT)
        self.waypoint_y_input = tk.IntVar()
        self.waypoint_y_input.set(GOAL_POS[1])
        tk.Spinbox(waypoint_frame, from_=0, to=NUM_ROWS - 1, textvariable=self.waypoint_y_input) \
            .pack(fill=tk.X, side=tk.LEFT)

        # Speed Setting
        speed_frame = tk.Frame(right_sidepanel,highlightbackground='black',highlightcolor="black", highlightthickness=1)
        speed_frame.pack(fill=tk.X, pady=10)

        self.create_heading(speed_frame, "Robot Speed (Moves Per Second)").pack(fill=tk.X)

        self.robot_speed = tk.IntVar()
        self.robot_speed.set(5)
        tk.Spinbox(speed_frame, from_=SimulatorGUI.MIN_SPEED, to=SimulatorGUI.MAX_SPEED, textvariable=self.robot_speed) \
            .pack(fill=tk.X)
        self.create_button(speed_frame, "Update Speed", self.update_speed).pack(fill=tk.X)

    @staticmethod
    def create_heading(master, text, is_small=False):
        return tk.Label(
            master,
            text=text,
            font=Font(family="Helvetica", size=16 if is_small else 18, weight="normal" if is_small else "bold"),
        )

    @staticmethod
    def create_button(master, text, command):
        return tk.Button(
            master,
            text=text,
            font=Font(family="Helvetica", size=12),
            command=command,
            pady=3,
        )

    def display_error_msg(self, text):
        label = tk.Label(self.error_frame, text=text, fg=GUI.RED)
        label.pack()
        label.after(3000, lambda: label.destroy())

    def update_speed(self):
        try:
            robot_speed = self.robot_speed.get()

            if SimulatorGUI.MIN_SPEED <= robot_speed <= SimulatorGUI.MAX_SPEED:
                self.robot.speed = robot_speed
            else:
                self.display_error_msg("Invalid speed")

        except Exception:
            self.display_error_msg("Invalid speed")

    def exploration(self):
        self.is_running = True
        self.reset()
        self.robot.map = self.map
        with_image_rec = self.with_image_rec.get() == 1

        # Select exploration class
        exploration_class = CompleteImageRecExploration if with_image_rec else Exploration

        self.exp = exploration_class(
            self.robot,
            self.update_canvas,
            coverage_limit=self.coverage_limit_input.get() / 100,
            time_limit=self.time_limit_input.get()
        )

        self.map = self.exp.explored_map
        self.update_canvas()
        self.exp.run_exploration()
        mdf = generate_map_descriptor(self.exp.explored_map)
        print("MDF:", ",".join(mdf))

        if with_image_rec:
            print(self.exp.obstacles)

        self.is_running = False

    def update_canvas(self):
        super(SimulatorGUI, self).update_canvas()
        #self.exploration_coverage.set("Coverage: {:.2f}%".format(self.exp.coverage * 100))
        #time_elapsed = round(self.exp.time_elapsed)

        #self.exploration_time.set("Time: {:02}:{:02}".format(time_elapsed // 60, time_elapsed % 60))

    def fastest_path(self):
        self.is_running = True
        self.reset(False)

        waypoint = self.get_waypoint()

        if waypoint is not None:
            self.waypoint = waypoint
            self.update_canvas()

        fp = FastestPath(self.map, Direction.EAST, START_POS, GOAL_POS, waypoint)

        if fp.path_found:
            for movement in fp.movements:
                if not self.is_running:
                    break

                self.robot.move(movement)

        else:
            print("NO PATH FOUND!")
            self.display_error_msg("No path found")

        self.is_running = False

    def reset(self, reset_map=True):
        self.waypoint = None

        if reset_map:
            self.map = self.selected_map.copy()
            self.update_map()

        self.robot.pos = START_POS
        self.robot.direction = Direction.EAST
        self.update_robot()

    def load_map(self):
        selected_map = SimulatorGUI.MAP_OPTIONS[self.selected_map_str.get()]

        if selected_map is not None:
            with open(selected_map, "r") as f:
                strs = f.read().split("\n")

            self.selected_map = generate_map(*strs)
            self.reset()

        else:
            try:
                self.selected_map = generate_map(*self.mdf_input.get().strip().split(","))
                self.reset()

            except (IndexError, ValueError):
                self.display_error_msg("Invalid Map Descriptor")

    def get_waypoint(self):
        if self.has_waypoint_input.get() != 1:
            return None

        try:
            x = int(self.waypoint_x_input.get())
            y = int(self.waypoint_y_input.get())

            if x < 0 or x >= NUM_COLS or y < 0 or y >= NUM_ROWS or self.map[y][x] != Cell.FREE:
                self.display_error_msg("Invalid Waypoint")
                return None

            return x, y

        except ValueError:
            self.display_error_msg("Invalid Waypoint")

        return None

    def execute_thread(self, method):
        if self.current_thread is not None and self.current_thread.is_alive():
            return

        self.current_thread = Thread(target=method, daemon=True)
        self.current_thread.start()

    def stop(self):
        if self.exp:
            self.exp.is_running = False
        self.is_running = False


if __name__ == '__main__':
    gui = SimulatorGUI()
    gui.start()
