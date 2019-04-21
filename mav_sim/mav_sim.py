from morse.builder import *

# creates a new instance of the robot
mav = Quadrotor()

# place mav at the correct location
mav.translate(0.0, 0.0, 2.0)
mav.rotate(0.0, 0.0, 0.0)

# create actuator
ft_actuator = ForceTorque()
mav.append(ft_actuator)

# create sensor
pose_sensor = Pose()
mav.append(pose_sensor)

# create another actuator
waypoint_actuator = RotorcraftWaypoint()
mav.append(waypoint_actuator)

# set up comms
pose_sensor.add_interface('socket')
ft_actuator.add_interface('socket')
waypoint_actuator.add_service('socket')
 
# set up environment
env = Environment('indoors-1/boxes')
env.set_camera_location([5, -5, 6])
env.set_camera_rotation([1.0470, 0, 0.7854])
