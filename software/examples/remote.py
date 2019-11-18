from pyluos import Robot
from orbita import Actuator
import time

a = Actuator()

r = Robot('/dev/cu.usbserial-DN05NM0L')
r.gate.delay = 10
r.disk_bottom.rot_position = False
r.disk_middle.rot_position = False
r.disk_top.rot_position = False

# ##########Luos Parameters############

r.disk_bottom.encoder_res = 5
r.disk_middle.encoder_res = 5
r.disk_top.encoder_res = 5


r.disk_bottom.setToZero()
r.disk_middle.setToZero()
r.disk_top.setToZero()


r.disk_bottom.reduction = 232
r.disk_middle.reduction = 232
r.disk_top.reduction = 232


r.disk_bottom.wheel_size = 60.0
r.disk_middle.wheel_size = 60.0
r.disk_top.wheel_size = 60


r.disk_bottom.positionPid = [9, 0.02, 100]
r.disk_middle.positionPid = [9, 0.02, 100]
r.disk_top.positionPid = [9, 0.02, 100]


r.disk_bottom.rot_position_mode(True)
r.disk_middle.rot_position_mode(True)
r.disk_top.rot_position_mode(True)

# ##Motors not compliant###

r.disk_bottom.compliant = False
r.disk_middle.compliant = False
r.disk_top.compliant = False

# ##Control the actuator with an IMU###

a.reset_last_angles()
while(True):
    imu_quat = r.Imu_mod.quaternion
    q11, q12, q13 = a.from_quaternion_get_angles(imu_quat)
    r.disk_top.target_rot_position = q11
    r.disk_middle.target_rot_position = q12
    r.disk_bottom.target_rot_position = q13

    time.sleep(0.01)
