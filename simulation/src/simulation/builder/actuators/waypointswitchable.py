from morse.builder.creator import ActuatorCreator

class Waypointswitchable(ActuatorCreator):
    _classpath = "simulation.actuators.waypointswitchable.Waypointswitchable"
    _blendname = "waypointswitchable"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

