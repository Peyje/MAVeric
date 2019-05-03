class Control:
    def __init__(self, x_path, y_path, z_path, phi_path):
        self.x_path = x_path
        self.y_path = y_path
        self.z_path = z_path
        self.phi_path = phi_path

    def nextUp(self, time, state):
        print("I bims", time, state.x, state.y, state.z)