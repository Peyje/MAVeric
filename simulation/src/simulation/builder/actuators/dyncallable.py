from morse.builder.creator import ActuatorCreator

class Dyncallable(ActuatorCreator):
    _classpath = "simulation.actuators.dyncallable.Dyncallable"
    _blendname = "dyncallable"

    def __init__(self, name=None):
        ActuatorCreator.__init__(self, name)

