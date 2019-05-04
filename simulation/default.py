from morse.builder import *
from simulation.builder.actuators import Dyncallable, Waypointswitchable

# creates a new instance of the robot
mav = Quadrotor()
#mav.set_mass(0.028) # real mass of Craziefly drone is too light for simulation

# place mav at an initial location
mav.translate(0.0, 0.0, 2)
mav.rotate(0.0, 0.0, 0.0)

# create sensor for pose
pose_sensor = Pose()
mav.append(pose_sensor)

# create sensor for velocity
velocity_sensor = Velocity()
mav.append(velocity_sensor)

# create actuator for dynamic flight (the extended one which is callable)
dyn_callable = Dyncallable()
mav.append(dyn_callable)

# create another actuator for setting waypoints
waypoint_actuator = Waypointswitchable()
mav.append(waypoint_actuator)

# set up comms
pose_sensor.add_interface('socket')
velocity_sensor.add_interface('socket')
dyn_callable.add_interface('socket')
waypoint_actuator.add_service('socket')
 
# set up environment
env = Environment('indoors-1/boxes')
# env.configure_stream_manager('socket', time_sync = True, sync_port = 12000) # time sync to GUI, not used anymore
env.set_camera_location([5, -5, 6])
env.set_camera_rotation([1.0470, 0, 0.7854])
