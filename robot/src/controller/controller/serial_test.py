from rosmaster import Rosmaster

# com = "COM30"
com = "/dev/ttyUSB0"
# com="/dev/ttyTHS1"
# com="/dev/ttyAMA0"
ros_master = Rosmaster(car_type=1, com=com, debug=True)
ros_master.set_motor(0, 0, 0, 0)
ros_master.set_beep(0)
