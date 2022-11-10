from controller import Robot


class MazeRunner(Robot):
    def __init__(self):
        super().__init__()
        # Get simulation step length.
        self.timeStep = int(self.getBasicTimeStep())

        self.distance_sensor_data = {
            "central": [0.0, "no_collision"],
            "central_left": [0.0, "no_collision"],
            "central_right": [0.0, "no_collision"],
            "outer_left": [0.0, "no_collision"],
            "outer_right": [0.0, "no_collision"]}

        self.closest_wall_direction = None
        self.central_wall_collision_threshold = 2700.0
        self.outer_wall_collision_threshold = 2500.0

        self.init_time = 0.0
        self.sensor_update_rate = 0.3

        self.robot_state = "halting"
        self.robot_previous_state = "halting"

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

        # Enable Motor Position sensors
        self.leftMotorPosition = self.getDevice("motor.left.position")
        self.rightMotorPosition = self.getDevice("motor.right.position")

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

    def update_sensor_data_value(self, new_measurement: float, current_measurement: float):
        return current_measurement * self.sensor_update_rate + new_measurement * (1 - self.sensor_update_rate)

    def robot_change_state(self, next_state: str):
        if self.robot_state != next_state:
            self.robot_previous_state = self.robot_state
            self.robot_state = next_state

    def get_and_print_distance_sensor_data(self, print_data=True):
        sensor_dict = {
            "central": self.centralSensor.getValue(),
            "central_left": self.centralLeftSensor.getValue(),
            "central_right": self.centralRightSensor.getValue(),
            "outer_left": self.outerLeftSensor.getValue(),
            "outer_right": self.outerRightSensor.getValue()}
        for key in self.distance_sensor_data.keys():
            self.distance_sensor_data[key][0] = sensor_dict[key]
        for key in self.distance_sensor_data.keys():
            if "central" in key:
                if self.distance_sensor_data[key][0] >= self.central_wall_collision_threshold:
                    self.distance_sensor_data[key][1] = "collision"
                else:
                    self.distance_sensor_data[key][1] = "no_collision"
            elif "outer" in key:
                if self.distance_sensor_data[key][0] >= self.outer_wall_collision_threshold:
                    self.distance_sensor_data[key][1] = "collision"
                else:
                    self.distance_sensor_data[key][1] = "no_collision"
        sorted_data = sorted(self.distance_sensor_data.items(), key=lambda kv: kv[1], reverse=True)
        self.closest_wall_direction = None
        for measure in sorted_data:
            if measure[1][0] > 0.0:
                self.closest_wall_direction = measure[0]
                break
        if print_data:
            print(self.distance_sensor_data)

    def robot_go(self):
        self.leftMotor.setPosition(float('inf'))
        self.rightMotor.setPosition(float('inf'))
        self.leftMotor.setVelocity(self.velocity)
        self.rightMotor.setVelocity(self.velocity)

    def robot_stop(self):
        self.leftMotor.setPosition(float(0.0))
        self.rightMotor.setPosition(float(0.0))

    def robot_back_of(self):
        self.leftMotor.setVelocity(-self.velocity)
        self.rightMotor.setVelocity(-self.velocity)

    def robot_turn_right(self):
        self.leftMotor.setVelocity(self.velocity)
        self.rightMotor.setVelocity(-self.velocity)

    def robot_turn_left(self):
        self.leftMotor.setVelocity(-self.velocity)
        self.rightMotor.setVelocity(self.velocity)

    def robot_u_turn(self, direction: str, turn_strength: float = 0.6):
        if direction == "right":
            self.leftMotor.setVelocity(self.velocity)
            self.rightMotor.setVelocity(self.velocity * turn_strength)
        elif direction == "left":
            self.leftMotor.setVelocity(self.velocity * turn_strength)
            self.rightMotor.setVelocity(self.velocity)

    def robot_decide_next_maneuver(self):
        if self.robot_state == "halting":
            self.robot_go()
            self.robot_change_state(next_state="driving")
        if "driving" in self.robot_state:
            ###### 8b. L/R
            if "hugging" in self.robot_state:
                if self.distance_sensor_data["central"][1] == "no_collision" \
                        and self.distance_sensor_data["central_left"][1] == "no_collision" \
                        and self.distance_sensor_data["central_right"][1] == "no_collision":
                    if self.distance_sensor_data["outer_left"][0] == 0.0 \
                            and self.robot_state == "driving_hugging_left_wall":
                        self.leftMotorInitPosition = self.leftMotorPosition.getValue()
                        self.rightMotorInitPosition = self.rightMotorPosition.getValue()
                        self.robot_change_state(next_state="approach_to_left_wall_edge")
                    elif self.distance_sensor_data["outer_right"][0] == 0.0 \
                            and self.robot_state == "driving_hugging_right_wall":
                        self.leftMotorInitPosition = self.leftMotorPosition.getValue()
                        self.rightMotorInitPosition = self.rightMotorPosition.getValue()
                        self.robot_change_state(next_state="approach_to_right_wall_edge")
            ######
            ###### 2. L/R
            if self.distance_sensor_data["central"][1] == "collision":
                if self.distance_sensor_data["outer_left"][0] > self.distance_sensor_data["outer_right"][0]:
                    self.robot_turn_right()
                    self.robot_change_state(next_state="turning_right")
                elif self.distance_sensor_data["outer_left"][0] < self.distance_sensor_data["outer_right"][0]:
                    self.robot_turn_left()
                    self.robot_change_state(next_state="turning_left")
            ######
            ###### 6. L/R
            elif self.distance_sensor_data["central"][1] == "no_collision" \
                    and (self.distance_sensor_data["central_left"][1] == "collision"
                         or self.distance_sensor_data["outer_left"][1] == "collision"):
                self.robot_turn_right()
                self.robot_change_state(next_state="turning_right")
            elif self.distance_sensor_data["central"][1] == "no_collision" \
                    and (self.distance_sensor_data["central_right"][1] == "collision"
                         or self.distance_sensor_data["outer_right"][1] == "collision"):
                self.robot_turn_left()
                self.robot_change_state(next_state="turning_left")
            ######
            ###### 8a. L/R
            elif self.distance_sensor_data["central"][1] == "no_collision" \
                    and self.distance_sensor_data["central_left"][1] == "no_collision" \
                    and self.distance_sensor_data["central_right"][1] == "no_collision":
                if self.distance_sensor_data["outer_left"][0] > self.distance_sensor_data["outer_right"][0]:
                    self.robot_change_state(next_state="driving_hugging_left_wall")
                elif self.distance_sensor_data["outer_left"][0] < self.distance_sensor_data["outer_right"][0]:
                    self.robot_change_state(next_state="driving_hugging_right_wall")
            ######
        elif self.robot_state == "turning_left" or self.robot_state == "turning_right":
            ###### 7.
            if self.distance_sensor_data["central"][1] == "no_collision" \
                    and self.distance_sensor_data["central_left"][1] == "no_collision" \
                    and self.distance_sensor_data["central_right"][1] == "no_collision" \
                    and self.distance_sensor_data["outer_left"][1] == "no_collision" \
                    and self.distance_sensor_data["outer_right"][1] == "no_collision":
                self.robot_go()
                self.robot_change_state(next_state="driving")
            ######
        elif self.robot_state == "approach_to_right_wall_edge" or self.robot_state == "approach_to_left_wall_edge":
            ###### 8c. L/R
            left = self.leftMotorPosition.getValue()
            right = self.rightMotorPosition.getValue()
            t1 = self.leftMotorInitPosition
            t2 = self.rightMotorInitPosition
            if self.distance_sensor_data["central"][1] == "no_collision" \
                    and self.distance_sensor_data["central_left"][1] == "no_collision" \
                    and self.distance_sensor_data["central_right"][1] == "no_collision" \
                    and self.distance_sensor_data["outer_left"][1] == "no_collision" \
                    and self.distance_sensor_data["outer_right"][1] == "no_collision":
                if self.leftMotorInitPosition + 6.0 <= left \
                        and self.rightMotorInitPosition + 6.0 <= right:
                    if self.robot_state == "approach_to_left_wall_edge":
                        self.robot_u_turn(direction="left", turn_strength=0.1)
                        self.robot_change_state(next_state="u_turning_left")
                    elif self.robot_state == "approach_to_right_wall_edge":
                        self.robot_u_turn(direction="right", turn_strength=0.1)
                        self.robot_change_state(next_state="u_turning_right")
            else:
                self.robot_go()
                self.robot_change_state(next_state="driving")
            ######
        elif self.robot_state == "u_turning_right" or self.robot_state == "u_turning_left":
            ###### 8d. L/R
            if self.distance_sensor_data["central"][1] == "collsion" \
                    or self.distance_sensor_data["central_left"][1] == "collision" \
                    or self.distance_sensor_data["central_right"][1] == "collision" \
                    and (self.distance_sensor_data["outer_left"][1] == "no_collision"
                         and self.distance_sensor_data["outer_right"][1] == "no_collision"):
                if self.robot_state == "u_turning_left":
                    self.robot_turn_left()
                    self.robot_change_state(next_state="turning_left")
                elif self.robot_state == "u_turning_right":
                    self.robot_turn_right()
                    self.robot_change_state(next_state="turning_right")
            elif self.distance_sensor_data["central"][1] == "no_collision" \
                    and self.distance_sensor_data["central_left"][1] == "no_collision" \
                    and self.distance_sensor_data["central_right"][1] == "no_collision" \
                    and (self.distance_sensor_data["outer_left"][1] == "collision"
                         or self.distance_sensor_data["outer_right"][1] == "collision"):
                self.robot_go()
                self.robot_change_state(next_state="driving")
            ######


maze_runner = MazeRunner()
while maze_runner.step(maze_runner.timeStep) != -1:
    maze_runner.get_and_print_distance_sensor_data(print_data=False)
    print(f"Closest Wall Direction: {maze_runner.closest_wall_direction}")
    print(f"Robot Current State: {maze_runner.robot_state}")
    maze_runner.robot_decide_next_maneuver()
    # print(next_maneuver)
    # if next_maneuver == "go_back":
    #     maze_runner.robot_back_of()
    # if next_maneuver == "turn_right":
    #     maze_runner.robot_turn_right()
    # if next_maneuver == "turn_left":
    #     maze_runner.robot_turn_left()
    # if next_maneuver == "go_forward":
    #     maze_runner.robot_stop_turning_and_go()
