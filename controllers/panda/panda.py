from controller import Robot


class Panda(Robot):
    def __init__(self):
        super().__init__()
        self.timeStep = int(self.getBasicTimeStep())

        self.panda_finger_joint1 = self.getDevice("panda_finger_joint1")
        self.panda_finger_joint2 = self.getDevice("panda_finger_joint2")

        self.panda_joint1 = self.getDevice("panda_joint1")
        self.panda_joint2 = self.getDevice("panda_joint2")
        self.panda_joint3 = self.getDevice("panda_joint3")
        self.panda_joint4 = self.getDevice("panda_joint4")
        self.panda_joint5 = self.getDevice("panda_joint5")
        self.panda_joint6 = self.getDevice("panda_joint6")
        self.panda_joint7 = self.getDevice("panda_joint7")


panda = Panda()
while panda.step(panda.timeStep) != -1:
    pass
