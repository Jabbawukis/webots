"""Naive maze runner controller."""
from controller import Robot


class MazeRunner(Robot):
    def __init__(self):
        super().__init__()
        # Get simulation step length.
        self.timeStep = int(self.getBasicTimeStep())

        self.central = 0.0
        self.central_left = 0.0
        self.outer_left = 0.0
        self.central_right = 0.0
        self.outer_right = 0.0
        self.wall_collision_threshold = 3500.0
        self.side_wall_collision_threshold = 2500.0
        self.collisions_detected = None
        self.black_circles_detected = 0
        self.black_circle_detected = False
        self.init_time = 0.0

        self.robot_state = "halting"
        self.robot_last_state = ""

        self.ground_left = 0.0
        self.ground_right = 0.0
        self.black_circle_threshold = 500.0

        # Constants of the Thymio II motors and distance sensors.
        self.maxMotorVelocity = 6

        # Get left and right wheel motors.
        self.leftMotor = self.getDevice("motor.left")
        self.rightMotor = self.getDevice("motor.right")

        # Frontal distance sensors that can be use to detect the walls.
        self.outerLeftSensor = self.getDevice("prox.horizontal.0")
        self.centralLeftSensor = self.getDevice("prox.horizontal.1")
        self.centralSensor = self.getDevice("prox.horizontal.2")
        self.centralRightSensor = self.getDevice("prox.horizontal.3")
        self.outerRightSensor = self.getDevice("prox.horizontal.4")

        # Enable sensors.
        self.outerLeftSensor.enable(self.timeStep)
        self.centralLeftSensor.enable(self.timeStep)
        self.centralSensor.enable(self.timeStep)
        self.centralRightSensor.enable(self.timeStep)
        self.outerRightSensor.enable(self.timeStep)

        # Get and enable ground sensors to detect the black circles.
        self.groundLeftSensor = self.getDevice("prox.ground.0")
        self.groundRightSensor = self.getDevice("prox.ground.1")
        self.groundLeftSensor.enable(self.timeStep)
        self.groundRightSensor.enable(self.timeStep)

        # Disable motor PID control mode.
        self.leftMotor.setPosition(float(0.0))
        self.rightMotor.setPosition(float(0.0))

        self.init_time = self.getTime()

        # Set ideal motor velocity.
        self.velocity = 0.7 * self.maxMotorVelocity

    def get_and_print_distance_sensor_data(self, print_data=True):
        self.collisions_detected = None
        collision = {}
        self.central = self.centralSensor.getValue()
        self.central_left = self.centralLeftSensor.getValue()
        self.outer_left = self.outerLeftSensor.getValue()
        self.central_right = self.centralRightSensor.getValue()
        self.outer_right = self.outerRightSensor.getValue()
        if self.central > 0.0:
            if self.central >= self.wall_collision_threshold:
                collision["central"] = self.central
            if print_data:
                print(f"centralSensor: {self.central}")
        if self.central_left > 0.0:
            if self.central_left >= self.wall_collision_threshold:
                collision["central_left"] = self.central_left
            if print_data:
                print(f"centralLeftSensor: {self.central_left}")
        if self.outer_left > 0.0:
            if self.outer_left >= self.side_wall_collision_threshold:
                collision["outer_left"] = self.outer_left
            if print_data:
                print(f"outerLeftSensor: {self.outer_left}")
        if self.central_right > 0.0:
            if self.central_right >= self.wall_collision_threshold:
                collision["central_right"] = self.central_right
            if print_data:
                print(f"centralRightSensor: {self.central_right}")
        if self.outer_right > 0.0:
            if self.outer_right >= self.side_wall_collision_threshold:
                collision["outer_right"] = self.outer_right
            if print_data:
                print(f"outerRightSensor: {self.outer_right}")
        if len(collision.keys()) > 0:
            self.robot_last_state = self.robot_state
            self.robot_state = "wall_collision_detected"
            self.collisions_detected = collision
        else:
            self.robot_last_state = self.robot_state
            self.robot_state = "going_forward"

    def get_and_print_ground_sensor_data(self, print_data=True):
        self.ground_left = self.groundLeftSensor.getValue()
        self.ground_right = self.groundRightSensor.getValue()
        if self.ground_left > 0.0 and print_data:
            print(f"groundLeftSensor: {self.ground_left}")
        if self.ground_right > 0.0 and print_data:
            print(f"groundRightSensor: {self.ground_right}")
        if self.ground_left <= self.black_circle_threshold or self.ground_right <= self.black_circle_threshold:
            self.init_time = self.getTime()
            self.black_circle_detected = True
        else:
            current_time = self.getTime()
            if self.black_circle_detected and current_time > self.init_time + 2.0:
                self.black_circle_detected = False
                self.black_circles_detected += 1
                print(self.black_circles_detected)

    def robot_go(self):
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
        self.robot_last_state = self.robot_state
        self.robot_state = "going_forward"

    def robot_stop(self):
        self.leftMotor.setPosition(float(0.0))
        self.rightMotor.setPosition(float(0.0))
        self.robot_last_state = self.robot_state
        self.robot_state = "halting"

    def robot_turn_right(self):
        self.robot_last_state = self.robot_state
        self.robot_state = "turning_right"
        self.leftMotor.setVelocity(self.velocity)
        self.rightMotor.setVelocity(-self.velocity)

    def robot_turn_left(self):
        self.robot_last_state = self.robot_state
        self.robot_state = "turning_left"
        self.leftMotor.setVelocity(-self.velocity)
        self.rightMotor.setVelocity(self.velocity)

    def robot_stop_turning(self):
        self.robot_last_state = self.robot_state
        self.robot_state = "going_forward"
        self.leftMotor.setVelocity(self.velocity)
        self.rightMotor.setVelocity(self.velocity)

    def robot_detect_open_space(self):
        smallest_distance = min(self.collisions_detected, key=self.collisions_detected.get)
        if "left" in smallest_distance:
            return "right"
        elif "right" in smallest_distance:
            return "left"
        elif "central" in smallest_distance and len(self.collisions_detected.keys()) == 1:
            return "right"


maze_runner = MazeRunner()
maze_runner.robot_go()
while maze_runner.step(maze_runner.timeStep) != -1:
    maze_runner.get_and_print_distance_sensor_data(print_data=False)
    maze_runner.get_and_print_ground_sensor_data()
    if maze_runner.robot_state == "wall_collision_detected" and maze_runner.robot_last_state == "going_forward":
        best_direction = maze_runner.robot_detect_open_space()
        if best_direction == "left":
            maze_runner.robot_turn_left()
        if best_direction == "right":
            maze_runner.robot_turn_right()
    if maze_runner.robot_state == "going_forward" and maze_runner.robot_last_state == "wall_collision_detected":
        maze_runner.robot_stop_turning()
        maze_runner.robot_go()
    if maze_runner.robot_state == "going_forward" and maze_runner.robot_last_state == "going_forward":
        maze_runner.robot_stop_turning()
        maze_runner.robot_go()
    if maze_runner.black_circles_detected == 5:
        current_time = maze_runner.getTime()
        if current_time > maze_runner.init_time + 5.0:
            maze_runner.robot_stop()
            break

