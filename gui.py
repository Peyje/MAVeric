import telnetlib
import gi
import trajectory_planner
import control
from numpy import *

gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from gi.repository import GLib

# class to save current state of MAV
class State:
	def __init__(self):
		self.x = 0.0
		self.y = 0.0
		self.z = 0.0
		self.phi = 0.0
		self.theta = 0.0
		self.psi = 0.0
		self.x_dot = 0.0
		self.y_dot = 0.0
		self.z_dot = 0.0
		self.p = 0.0
		self.q = 0.0
		self.r = 0.0

# this class saves the current state of the MAV
class Bridge:
	def __init__(self):
		# retrieve labels for state
		self.state_x_label = builder.get_object("state_x")
		self.state_y_label = builder.get_object("state_y")
		self.state_z_label = builder.get_object("state_z")
		self.state_phi_label = builder.get_object("state_phi")
		self.state_theta_label = builder.get_object("state_theta")
		self.state_psi_label = builder.get_object("state_psi")
		self.state_x_dot_label = builder.get_object("state_x_dot")
		self.state_y_dot_label = builder.get_object("state_y_dot")
		self.state_z_dot_label = builder.get_object("state_z_dot")
		self.state_p_label = builder.get_object("state_p")
		self.state_q_label = builder.get_object("state_q")
		self.state_r_label = builder.get_object("state_r")

		# refresh those values every 100 ms
		#GLib.timeout_add(50, self.updateState)
		self.updateState()

	# update state values from telnet connection
	def updateState(self):
		# ask for current pose data
		comm.write(b'id1 mav.pose_sensor get_local_data \n')
		# update x value
		comm.read_until(b'"x": ')  # b'' as Telnet needs a bytes object instead of string since Python3
		read = comm.read_until(b',')  # returns read values + finishing ','
		read = read[:-1]  # cut that ','
		current_state.x = float(read)
		self.state_x_label.set_text("%0.2f" % current_state.x)
		# update y value
		comm.read_until(b'"y": ')
		read = comm.read_until(b',')
		read = read[:-1]
		current_state.y = float(read)
		self.state_y_label.set_text("%0.2f" % current_state.y)
		# update z value
		comm.read_until(b'"z": ')
		read = comm.read_until(b',')
		read = read[:-1]
		current_state.z = float(read)
		self.state_z_label.set_text("%0.2f" % current_state.z)
		# update yaw value
		comm.read_until(b'"yaw": ')
		read = comm.read_until(b',')
		read = read[:-1]
		current_state.psi = float(read)
		self.state_psi_label.set_text("%0.2f" % current_state.psi)
		# update pitch value
		comm.read_until(b'"pitch": ')
		read = comm.read_until(b',')
		read = read[:-1]
		current_state.theta = float(read)
		self.state_theta_label.set_text("%0.2f" % current_state.theta)
		# update roll value
		comm.read_until(b'"roll": ')
		read = comm.read_until(b'}')
		read = read[:-1]
		current_state.phi = float(read)
		self.state_phi_label.set_text("%0.2f" % current_state.phi)

		#ask for current velocity data
		comm.write(b'id1 mav.velocity_sensor get_local_data \n')
		#update p value
		comm.read_until(b'"angular_velocity": [')
		read = comm.read_until(b',')
		read = read[:-1]
		current_state.p = float(read)
		self.state_p_label.set_text("%0.2f" % current_state.p)
		# update q value
		read = comm.read_until(b',')
		read = read[:-1]
		current_state.q = float(read)
		self.state_q_label.set_text("%0.2f" % current_state.q)
		# update r value
		read = comm.read_until(b']')
		read = read[:-1]
		current_state.r = float(read)
		self.state_r_label.set_text("%0.2f" % current_state.r)

		# update x_dot value
		comm.read_until(b'"world_linear_velocity": [')
		read = comm.read_until(b',')
		read = read[:-1]
		current_state.x_dot = float(read)
		self.state_x_dot_label.set_text("%0.2f" % current_state.x_dot)
		# update y_dot value
		read = comm.read_until(b',')
		read = read[:-1]
		current_state.y_dot = float(read)
		self.state_y_dot_label.set_text("%0.2f" % current_state.y_dot)
		# update z_dot value
		read = comm.read_until(b']')
		read = read[:-1]
		current_state.z_dot = float(read)
		self.state_z_dot_label.set_text("%0.2f" % current_state.z_dot)

		return GLib.SOURCE_CONTINUE

# this class saves the current trajectory and invokes the following of said trajectory
class Trajectory:
	def __init__(self, planner_out):
		# extract data from planner output
		self.waypoint0 = planner_out[0]
		self.waypoint1 = planner_out[1]
		self.trajectory = planner_out[2]

		# create array of times (every 50ms)
		self.t = linspace(self.waypoint0.time, self.waypoint1.time, (self.waypoint1.time - self.waypoint0.time) * 20)

		# control variable for update loop
		self.i = 0

		# set up control object
		self.control = control.Control(self.trajectory, self.t)

	# start following trajectory
	def start(self):
		GLib.timeout_add(50, self.updateSpeeds)
		self.updateSpeeds()

	# calculate and set speed for rotors
	def updateSpeeds(self):
		# if end is reached (and time reached end.time) stop calling
		if self.i == self.t.shape[0]:
			return False

		# calculate next speeds
		speeds = self.control.nextUpEasy(self.i, current_state, self.waypoint1)

		# comm them TODO: UnCOMMent
		#command_string = 'id1 mav.dyn_callable setspeed [%s, %s, %s, %s] \n' % (speeds[0], speeds[1], speeds[2], speeds[3])
		#comm.write(bytes(command_string, 'utf8'))

		self.i = self.i + 1
		return GLib.SOURCE_CONTINUE

# handler class for GUI
class Handler:
	def __init__(self):
		# get GUI objects for GoTo
		self.goto_x_entry = builder.get_object("goto_x")
		self.goto_y_entry = builder.get_object("goto_y")
		self.goto_z_entry = builder.get_object("goto_z")
		self.goto_phi_entry = builder.get_object("goto_phi")

		# get GUI objects for Trajectory
		self.traj_to_x_entry = builder.get_object("traj_to_x")
		self.traj_to_y_entry = builder.get_object("traj_to_y")
		self.traj_to_z_entry = builder.get_object("traj_to_z")
		self.traj_to_phi_entry = builder.get_object("traj_to_phi")

		# get Go button from Trajectory Planner to set sensitive after calculating
		self.button_go_traj = builder.get_object("traj_go_button")

		# get Toggle Hold button to show corresponding status
		self.button_hold = builder.get_object("hold_button")
		self.hold = True

		self.trajectory = None

	def onGoToButtonPress(self, button):
		goto_x = self.goto_x_entry.get_text()
		goto_y = self.goto_y_entry.get_text()
		goto_z = self.goto_z_entry.get_text()
		goto_phi = self.goto_phi_entry.get_text()
		command_string = 'id1 mav.waypoint_actuator setdest [%s, %s, %s, %s, 0.2] \n' % (goto_x, goto_y, goto_z, goto_phi)
		comm.write(bytes(command_string, 'utf8'))

	def onTrajCalcButtonPress(self, button):
		traj_x = float(self.traj_to_x_entry.get_text())
		traj_y = float(self.traj_to_y_entry.get_text())
		traj_z = float(self.traj_to_z_entry.get_text())
		traj_phi = float(self.traj_to_phi_entry.get_text())

		# calculate trajectory and save as new Trajectory object
		self.trajectory = Trajectory(trajectory_planner.planner(current_state, traj_x, traj_y, traj_z, traj_phi))
		# enable go button
		self.button_go_traj.set_sensitive(True)

	def onTrajGoButtonPress(self, button):
		self.trajectory.start()

	def onSwitchActivate(self, button, state):
		command_string = 'id1 mav.waypoint_actuator switch_hold \n'
		comm.write(bytes(command_string, 'utf8'))


# MAIN
if __name__ == "__main__":
	# open telnet connection to port where interaction with sensors and actuators is possible
	comm = telnetlib.Telnet("localhost", 4000)

	# load layout from glade file
	builder = Gtk.Builder()
	builder.add_from_file("layout.glade")
	builder.connect_signals(Handler())

	# main GUI code
	window = builder.get_object("main")
	window.connect("destroy", Gtk.main_quit)
	window.show_all()

	# create state object
	current_state = State()

	# create object to combine GUI, MORSE and state object
	bridge = Bridge()

	# run GUI
	Gtk.main()
	# everything beyond this line will only be run after exiting!
