import telnetlib
import gi
import trajectory_planner
import control
from numpy import *

gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from gi.repository import GLib

# variable for radio switch of trajectory
radioJoint = True

# class to save current state of MAV into
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


# this class retrieves and stores the current state of MAV
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

		# refresh those values every 50 ms
		GLib.timeout_add(50, self.updateState)
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

		# update first waypoint for trajectory in GUI
		waypoints_gui[0] = [0, current_state.x, current_state.y, current_state.z, current_state.psi]

		return GLib.SOURCE_CONTINUE


# this class saves the current trajectory and invokes the following of said trajectory
class Trajectory:
	def __init__(self, planner_out):
		# extract data from planner output
		self.waypoints = planner_out[0]
		self.trajectory = planner_out[1]

		# create array of times (every 50ms)
		self.numSteps = (self.waypoints[-1].time - self.waypoints[0].time) * 20

		# control variable for update loop
		self.i = 0

		# set up control object
		self.control = control.Control(self.waypoints, self.trajectory)

	# start following trajectory
	def start(self):
		GLib.timeout_add(50, self.updateSpeeds)
		self.updateSpeeds()

	# calculate and set speed for rotors
	def updateSpeeds(self):
		# if end is reached stop calling
		if self.i == self.numSteps:
			# deactivate Go button
			button_go_traj = builder.get_object("traj_go_button")
			button_go_traj.set_sensitive(False)
			return False

		# controller
		point = self.control.nextUpPD(self.i)
		command_string = 'id1 mav.waypoint_actuator setdest [%s, %s, %s, %s, 0.2] \n' % (point[0], point[1], point[2], point[3])
		comm.write(bytes(command_string, 'utf8'))

		self.i = self.i + 1
		return GLib.SOURCE_CONTINUE


# handler class for GUI
class Handler:
	def __init__(self):
		# get GUI objects for GoTo
		self.goto_x_entry = builder.get_object("goto_x")
		self.goto_y_entry = builder.get_object("goto_y")
		self.goto_z_entry = builder.get_object("goto_z")
		self.goto_yaw_entry = builder.get_object("goto_yaw")

		# get GUI objects for Trajectory
		self.traj_to_x_entry = builder.get_object("traj_to_x")
		self.traj_to_y_entry = builder.get_object("traj_to_y")
		self.traj_to_z_entry = builder.get_object("traj_to_z")
		self.traj_to_yaw_entry = builder.get_object("traj_to_yaw")

		# get Go button from Trajectory Planner to set sensitive after calculating
		self.button_go_traj = builder.get_object("traj_go_button")

		# get Toggle Hold button to show corresponding status
		self.button_hold = builder.get_object("hold_button")
		self.hold = True

		self.trajectory = None  # no trajectory calculated at first

	# if PD Go button is pressed, get values and fly there
	def onGoToButtonPress(self, button):
		goto_x = self.goto_x_entry.get_text()
		goto_y = self.goto_y_entry.get_text()
		goto_z = self.goto_z_entry.get_text()
		goto_yaw = self.goto_yaw_entry.get_text()
		command_string = 'id1 mav.waypoint_actuator setdest [%s, %s, %s, %s, 0.2] \n' % (goto_x, goto_y, goto_z, goto_yaw)
		comm.write(bytes(command_string, 'utf8'))

	# if Add button is pressed, get values and add waypoint to list
	def onAddButtonPress(self, button):
		wp_x = float(self.traj_to_x_entry.get_text())
		wp_y = float(self.traj_to_y_entry.get_text())
		wp_z = float(self.traj_to_z_entry.get_text())
		wp_yaw = float(self.traj_to_yaw_entry.get_text())

		# add waypoint to list
		waypoints_gui.append([size(waypoints_gui), wp_x, wp_y, wp_z, wp_yaw])

		# reset entry fields
		self.traj_to_x_entry.set_text('')
		self.traj_to_y_entry.set_text('')
		self.traj_to_z_entry.set_text('')
		self.traj_to_yaw_entry.set_text('')

	# if Calculate button is pressed, calculate and create a trajectory object
	def onTrajCalcButtonPress(self, button):
		# calculate trajectory and save as new Trajectory object
		self.trajectory = Trajectory(trajectory_planner.planner(waypoints_gui, radioJoint))
		# enable go button
		self.button_go_traj.set_sensitive(True)

	# if Go button of Trajectory is pressed, start flying and reset list of waypoints
	def onTrajGoButtonPress(self, button):
		self.trajectory.start()
		# reset waypoints_gui
		waypoints_gui.clear()
		waypoints_gui.append([0, 0, 0, 2, 0])

	# switch decides if PD should try to hold position or not
	def onSwitchActivate(self, button, state):
		command_string = 'id1 mav.waypoint_actuator switch_hold \n'
		comm.write(bytes(command_string, 'utf8'))

	# check if trajectory should be calculated jointly or separate
	def onRadioJoint(self, button):
		global radioJoint
		radioJoint = not radioJoint


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

	# get object that shows current waypoints set for trajectory
	waypoints_gui = builder.get_object('waypoint_store')
	waypoints_gui.append([0,0,0,2,0])

	# create state object
	current_state = State()

	# create object to combine GUI, MORSE and state object
	bridge = Bridge()

	# run GUI
	Gtk.main()
	# everything beyond this line will only be run after exiting!
