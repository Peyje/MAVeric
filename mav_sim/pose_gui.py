import telnetlib
import gi

gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from gi.repository import GLib

# data fields for state
state_x = 0.0
state_y = 0.0
state_z = 0.0
state_phi = 0.0
state_theta = 0.0
state_psi = 0.0
state_x_dot = 0.0
state_y_dot = 0.0
state_z_dot = 0.0
state_p = 0.0
state_q = 0.0
state_r = 0.0

# this class saves the current state of the MAV
class State:
	def __init__(self):
		# retrieve labels
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

		# refresh those values every second
		GLib.timeout_add(100, self.updateState)
		self.updateState()

	# update state values from telnet connection
	def updateState(self):
		# ask for current pose data
		comm.write(b'id1 mav.pose_sensor get_local_data \n')
		# update x value
		comm.read_until(b'"x": ')  # b'' as Telnet needs a bytes object instead of string since Python3
		read = comm.read_until(b',')  # returns read values + finishing ','
		read = read[:-1]  # cut that ','
		global state_x
		state_x = float(read)
		self.state_x_label.set_text("%0.2f" % state_x)
		# update y value
		comm.read_until(b'"y": ')
		read = comm.read_until(b',')
		read = read[:-1]
		global state_y
		state_y = float(read)
		self.state_y_label.set_text("%0.2f" % state_y)
		# update z value
		comm.read_until(b'"z": ')
		read = comm.read_until(b',')
		read = read[:-1]
		global state_z
		state_z = float(read)
		self.state_z_label.set_text("%0.2f" % state_z)
		# update yaw value
		comm.read_until(b'"yaw": ')
		read = comm.read_until(b',')
		read = read[:-1]
		global state_psi
		state_psi = float(read)
		self.state_psi_label.set_text("%0.2f" % state_psi)
		# update pitch value
		comm.read_until(b'"pitch": ')
		read = comm.read_until(b',')
		read = read[:-1]
		global state_theta
		state_theta = float(read)
		self.state_theta_label.set_text("%0.2f" % state_theta)
		# update roll value
		comm.read_until(b'"roll": ')
		read = comm.read_until(b'}')
		read = read[:-1]
		global state_phi
		state_phi = float(read)
		self.state_phi_label.set_text("%0.2f" % state_phi)

		#ask for current velocity data
		comm.write(b'id1 mav.velocity_sensor get_local_data \n')
		#update p value
		comm.read_until(b'"angular_velocity": [')
		read = comm.read_until(b',')
		read = read[:-1]
		global state_p
		state_p = float(read)
		self.state_p_label.set_text("%0.2f" % state_p)
		# update q value
		read = comm.read_until(b',')
		read = read[:-1]
		global state_q
		state_q = float(read)
		self.state_q_label.set_text("%0.2f" % state_q)
		# update r value
		read = comm.read_until(b']')
		read = read[:-1]
		global state_r
		state_r = float(read)
		self.state_r_label.set_text("%0.2f" % state_r)

		# update x_dot value
		comm.read_until(b'"world_linear_velocity": [')
		read = comm.read_until(b',')
		read = read[:-1]
		global state_x_dot
		state_x_dot = float(read)
		self.state_x_dot_label.set_text("%0.2f" % state_x_dot)
		# update y_dot value
		read = comm.read_until(b',')
		read = read[:-1]
		global state_y_dot
		state_y_dot = float(read)
		self.state_y_dot_label.set_text("%0.2f" % state_y_dot)
		# update z_dot value
		read = comm.read_until(b']')
		read = read[:-1]
		global state_z_dot
		state_z_dot = float(read)
		self.state_z_dot_label.set_text("%0.2f" % state_z_dot)

		return GLib.SOURCE_CONTINUE


# handler class for GUI
class Handler:
	def __init__(self):
		# get GUI objects
		self.goto_x_entry = builder.get_object("goto_x")
		self.goto_y_entry = builder.get_object("goto_y")
		self.goto_z_entry = builder.get_object("goto_z")
		self.goto_phi_entry = builder.get_object("goto_phi")

	def onGoToButtonPress(self, button):
		goto_x = self.goto_x_entry.get_text()
		goto_y = self.goto_y_entry.get_text()
		goto_z = self.goto_z_entry.get_text()
		goto_phi = self.goto_phi_entry.get_text()
		command_string = 'id1 mav.waypoint_actuator setdest [%s, %s, %s, %s, 0.2] \n' % (goto_x, goto_y, goto_z, goto_phi)
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

	# run GUI
	Gtk.main()
	# everything beyond this line will only be run after exiting!
