from controller import Supervisor
import math
import random

class SupervisorLight:
    def __init__(self):
        self.time_step = 32
        self.threshold = 0.20  # Only XZ distance, no Y

        self.supervisor = Supervisor()

        # Robot position
        self.robot = self.supervisor.getFromDef("Controller")
        self.robot_pos = self.robot.getField("translation")

        # Light data
        self.light_locations = []
        self.light_on_fields = []

        for i in range(1, 5):
            node = self.supervisor.getFromDef(f"Light{i}")
            if node is None:
                print(f"ERROR: Light{i} not found")
                continue

            # SpotLight has a 'location' field
            loc = node.getField("location").getSFVec3f()
            self.light_locations.append(loc)

            # 'on' field to toggle
            self.light_on_fields.append(node.getField("on"))

        # Activate one light at start
        self.active = random.randint(0, 3)
        self.set_lights(self.active)

    def run(self):
        while self.supervisor.step(self.time_step) != -1:

            # Robot world position
            rpos = self.robot_pos.getSFVec3f()
            rx, rz = rpos[0], rpos[2]

            # Current target light
            lx, _, lz = self.light_locations[self.active]  # ignore Y

            # Distance in XZ plane
            dx = rx - lx
            dz = rz - lz
            distance = math.sqrt(dx*dx + dz*dz)

            # print(f"DIST: {distance:.3f}")

            # Check arrival
            if distance < self.threshold:
                old = self.active
                new = random.randint(0, 3)

                while new == old:
                    new = random.randint(0, 3)

                print(f"Reached Light {old+1} → Switching to Light {new+1}")
                self.active = new
                self.set_lights(self.active)

    def set_lights(self, index):
        for i, field in enumerate(self.light_on_fields):
            field.setSFBool(i == index)

if __name__ == "__main__":
    SupervisorLight().run()