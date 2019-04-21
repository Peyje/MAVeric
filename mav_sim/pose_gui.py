import sys
import telnetlib
import gi
gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from gi.repository import GLib

# main window of GUI
class LabelWindow(Gtk.Window):
	def __init__(self):
		Gtk.Window.__init__(self, title="Label Example")

# frame wherein pose values are written
class Frame(Gtk.Frame):
	def __init__(self):
		Gtk.Frame.__init__(self)
		self.set_label("Pose")

		grid = Gtk.Grid()
		self.add(grid)

		# add label for x value
		self.x_label = Gtk.Label("X")
		grid.add(self.x_label)
		#self.add(self.x_label)
		self.y_label = Gtk.Label("Y")
		grid.attach(self.y_label, 1, 0, 1, 1)
		#self.add(self.y_label)
		self.z_label = Gtk.Label("Z")
		grid.attach(self.z_label, 2, 0, 1, 1)
		#self.add(self.z_label)

		# refresh those values every second
		GLib.timeout_add_seconds(1, self.updatePose)
		self.updatePose()

	# update pose values from telnet connection
	def updatePose(self):
		# update x value
		tn.read_until('"x": ')
		pose_x = tn.read_until(',') # returns read values + finishing ,
		pose_x = pose_x[:-1] # cut that ,
		self.x_label.set_text("X: " + pose_x)
		# update y value
		tn.read_until('"y": ')
		pose_y = tn.read_until(',')
		pose_y = pose_y[:-1]
		self.y_label.set_text("Y: " + pose_y)
		#update z value
		tn.read_until('"z": ')
		pose_z = tn.read_until(',')
		pose_z = pose_z[:-1]
		self.z_label.set_text("Z: " + pose_z)
		return GLib.SOURCE_CONTINUE

if __name__ == "__main__":
	# open telnet connection to port where pose sensor streams to
	tn = telnetlib.Telnet("localhost", 60001)

	# main GUI code
	window = LabelWindow()        
	window.connect("destroy", Gtk.main_quit)
	window.set_border_width(30)
	window.add(Frame())
	window.show_all()
	Gtk.main()



