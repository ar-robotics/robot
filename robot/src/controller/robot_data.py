import time

from controller.robot import Robot


robot = Robot()

robot.ros_master.create_receive_threading()


while True:
    accelerometer_data = robot.ros_master.get_accelerometer_data()
    gyroscope_data = robot.ros_master.get_gyroscope_data()
    magnetometer_data = robot.ros_master.get_magnetometer_data()
    motion_data = robot.ros_master.get_motion_data()
    voltage_data = robot.ros_master.get_battery_voltage()

    print("---")
    print(f"{accelerometer_data=}")
    print(f"{gyroscope_data=}")
    print(f"{magnetometer_data=}")
    print(f"{motion_data=}")
    print(f"{voltage_data=}")
    time.sleep(0.2)
