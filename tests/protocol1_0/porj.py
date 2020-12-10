import cv2
import numpy as np
import time
from dynamixel_sdk import *
import os

# servo IDs
DXL_IDS = [4, 12, 16, 18]
# initial q of servos, [0-deg, 300-deg] -> [0, 1023], data-type int
INIT_POS = [825, 825, 512, 512]
DEVICENAME = 'COM9'

'''
DH parameters, in milimeters, data-type float
r?i -> the ith joint, rotate ? axis
t?i -> the ith joint, translate ? axis
'''
dh_zero = 0.0
DHPARAM = {
    'tz1': 0.0, 'tx1': dh_zero, 'rx1': -np.pi,
    'tz2': dh_zero, 'tx2': 0.0, 'rx2': dh_zero,
    'tz3': dh_zero, 'tx3': 0.0, 'rx3': dh_zero,
    'tz4': dh_zero, 'tx4': dh_zero, 'rx4': np.pi,
    'tz5': dh_zero, 'tx5': 0.0, 'rx5': dh_zero
}

'''
2D PLANE parameters, in milimeters, data-type float
LUC -> left up corner w.r.t. {w} in homogeneous transfermation matrix
WIDTH
HEIGHT
'''
PLANE = {
    'LUC': np.array([
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 1]
    ]),
    'WIDTH': 0,
    'HEIGHT': 0
}

'''
3D BALL parameters, in milimeters, data-type float
CENTER -> center of ball w.r.t. {w} in homogeneous transfermation matrix
RADIUS
'''
BALL = {
    'CENTER': np.array([
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 1]
    ]),
    'RADIUS': 0
}


class Servo:
    def __init__(
        self,
        ADDR_MX_TORQUE_ENABLE=24,
        ADDR_MX_GOAL_POSITION=30,
        ADDR_MX_PRESENT_POSITION=36,
        PROTOCOL_VERSION=1.0,
        DXL_IDS=None,
        INIT_POS=None,
        BAUDRATE=1000000,
        DEVICENAME=None,
        TORQUE_ENABLE=1,
        TORQUE_DISABLE=0,
        DXL_MINIMUM_POSITION_VALUE=0,
        DXL_MAXIMUM_POSITION_VALUE=1023,
        DXL_MOVING_STATUS_THRESHOLD=3,
        COMM_SUCCESS=0,
        COMM_TX_FAIL=-1001,
        init_b4_del=True,
        **kargs
    ):
        assert DXL_IDS and INIT_POS and DEVICENAME
        assert len(INIT_POS) == len(DXL_IDS)
        self.ADDR_MX_TORQUE_ENABLE = ADDR_MX_TORQUE_ENABLE
        self.ADDR_MX_GOAL_POSITION = ADDR_MX_GOAL_POSITION
        self.ADDR_MX_PRESENT_POSITION = ADDR_MX_PRESENT_POSITION
        self.PROTOCOL_VERSION = PROTOCOL_VERSION
        self.DXL_IDS = DXL_IDS
        self.INIT_POS = INIT_POS
        self.BAUDRATE = BAUDRATE
        self.DEVICENAME = DEVICENAME
        self.TORQUE_ENABLE = TORQUE_ENABLE
        self.TORQUE_DISABLE = TORQUE_DISABLE
        self.DXL_MINIMUM_POSITION_VALUE = DXL_MINIMUM_POSITION_VALUE
        self.DXL_MAXIMUM_POSITION_VALUE = DXL_MAXIMUM_POSITION_VALUE
        self.DXL_MOVING_STATUS_THRESHOLD = DXL_MOVING_STATUS_THRESHOLD
        self.COMM_SUCCESS = COMM_SUCCESS
        self.COMM_TX_FAIL = COMM_TX_FAIL
        self.init_b4_del = init_b4_del
        self.portHandler = PortHandler(self.DEVICENAME)
        self.packetHandler = PacketHandler(self.PROTOCOL_VERSION)
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            quit()
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            quit()
        self.servo_torque(self.DXL_IDS, self.TORQUE_ENABLE)
        self.write(self.DXL_IDS, self.INIT_POS, closed_loop=True)

    def __del__(self):
        if self.init_b4_del:
            self.write(self.DXL_IDS, self.INIT_POS, closed_loop=True)
        self.servo_torque(self.DXL_IDS, self.TORQUE_DISABLE)
        self.portHandler.closePort()

    def servo_torque(self, DXL_IDS, type):
        if type == self.TORQUE_ENABLE:
            str_type = 'enabled'
        else:
            str_type = 'disabled'
        for DXL_ID in DXL_IDS:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
                self.portHandler, DXL_ID, self.ADDR_MX_TORQUE_ENABLE, type)
            if dxl_comm_result != self.COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel#%d torque %s" % (DXL_ID, str_type))

    def read(self, DXL_IDS):
        return [self.packetHandler.read4ByteTxRx(
            self.portHandler, DXL_ID, self.ADDR_MX_PRESENT_POSITION
        )[0] for DXL_ID in DXL_IDS]

    def write(self, DXL_IDS, POS, closed_loop=False):
        assert len(DXL_IDS) == len(POS)
        for DXL_ID, pos in zip(DXL_IDS, POS):
            dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
                self.portHandler, DXL_ID, self.ADDR_MX_GOAL_POSITION, pos)
            if dxl_comm_result != self.COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        while closed_loop and (np.abs(np.array(self.read(DXL_IDS)) - np.array(POS)) >
                               self.DXL_MOVING_STATUS_THRESHOLD).any():
            pass


class Robot:
    def __init__(self, DXL_IDS, INIT_POS, DEVICENAME,
                 DHPARAM, PLANE, BALL,
                 **kargs):
        self.servo = Servo(DXL_IDS=DXL_IDS, INIT_POS=INIT_POS,
                           DEVICENAME=DEVICENAME, **kargs)
        self.DHPARAM = DHPARAM
        self.PLANE = PLANE
        self.BALL = BALL

    def calibration(self):
        # do not need to implement now
        pass

    def process_input(self):
        def image_processing(img):
            '''
            @input: 
            img is a raw image in cv2-image object
            @output:
            a cv2-image object of thin lines of edges
            @parameters:
            None
            '''
            pass

        def img2eoe_path(img, draw_type):
            '''
            @input:
            img is a processed cv2-image object
            darw_type is to distinguish plane or ball
            @output:
            a list of tuples in (x, y, z, ox, oy, oz) w.r.t. {w} in order
            o* stand for the * axis part of orientation vector
            x, y, z in milimeters, data-type float
            ox, oy, oz is normalized
            @parameters:
            self.PLANE and self.BALL, refer to PLANE and BALL
            '''
            pass
        img_file = input('Please input image path: ')
        draw_type = None
        while draw_type != '2D' or draw_type != '3D':
            draw_type = input('Please input draw type(2D/3D): ')
        img = cv2.imread(img_file)
        img = image_processing(img, draw_type)
        return img2eoe_path(img)

    def move(self, eoe_path):
        def ik(eoe_path):
            '''
            @input:
            eoe_path is the emd of effector path, refer to img2eoe_path
            @output:
            a list of tuples in (q1, q2, q3, q4, q5), refer to INIT_POS
            @parameters:
            self.DHPARAM, refer to DHPARAM
            '''
            pass
        joint_path = ik(eoe_path)
        for point in joint_path:
            self.servo.write(self.servo.DXL_IDS, point, closed_loop=True)


robot = Robot(
    DXL_IDS=DXL_IDS,
    INIT_POS=INIT_POS,
    DEVICENAME=DEVICENAME,
    DHPARAM=DHPARAM,
    PLANE=PLANE,
    BALL=BALL
)
# robot.move(robot.process_input())

print(robot.servo.read(robot.servo.DXL_IDS))

t0 = time.time()

for i, x in enumerate(range(512, 302, -5)):
    t1 = time.time()
    while t1 - t0 > 0.1:
        pass
    robot.servo.write(robot.servo.DXL_IDS,
                      [825 - 5 * i, 825 - 5 * i, x, 512])
    t0 = t1
