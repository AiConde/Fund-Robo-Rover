#!/usr/bin/env python3

import time

from pySerialTransfer import pySerialTransfer as serial_transfer
import rospy
from std_msgs.msg import Float32, Int32, Time, UInt32
from common_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion

from dataclasses import dataclass

@dataclass
class cmd_frame:
    esc_pwm: float = 0.0
    pan_servo: float = 0.0
    steer_servo: float = 0.0
    sys_reset: bool = False
    transfer_success: bool = True

@dataclass
class status_frame:
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0
    mag_x: float = 0.0
    mag_y: float = 0.0
    mag_z: float = 0.0
    lsm9_tstamp: int = 0

    sharpir1: float = 0.0
    sharpir2: float = 0.0
    sharpir3: float = 0.0
    sharpir4: float = 0.0
    sharpir5: float = 0.0
    sharpir6: float = 0.0
    sharpir_tstamp: int = 0

    sonar1: float = 0.0
    sonar2: float = 0.0
    sonar_tstamp: int = 0

    tacometer_count: int = 0
    tacometer_tstamp: int = 0

try:
    link = serial_transfer.SerialTransfer('/dev/ttyACM0')
    link.open()
except:                  
    import traceback     
    traceback.print_exc()
    try:                 
        link.close()     
    except:              
        pass             
    import sys           
    sys.exit(0)          

time.sleep(1)


def txrx(pwm, pan, steer):

    cmd_frame_instance = cmd_frame()
    cmd_frame_instance.esc_pwm = pwm
    cmd_frame_instance.pan_servo = pan
    cmd_frame_instance.steer_servo = steer

    status_frame_instance = status_frame()

    send_size = 0
    send_size = link.tx_obj(cmd_frame_instance.esc_pwm, start_pos=send_size)
    send_size = link.tx_obj(cmd_frame_instance.pan_servo, start_pos=send_size)
    send_size = link.tx_obj(cmd_frame_instance.steer_servo, start_pos=send_size)
    send_size = link.tx_obj(cmd_frame_instance.sys_reset, start_pos=send_size)
    send_size = link.tx_obj(cmd_frame_instance.transfer_success, start_pos=send_size)

    link.send(send_size)

    while not link.available():
        if link.status < 0:
            if link.status == txfer.CRC_ERROR:
                print('ERROR: CRC_ERROR')
            elif link.status == txfer.PAYLOAD_ERROR:
                print('ERROR: PAYLOAD_ERROR')
            elif link.status == txfer.STOP_BYTE_ERROR:
                print('ERROR: STOP_BYTE_ERROR')
            else:
                print('ERROR: {}'.format(link.status))
    
    rx_size = 0
    status_frame_instance.accel_x = link.rx_obj(obj_type='f', start_pos=rx_size)
    rx_size += serial_transfer.STRUCT_FORMAT_LENGTHS['f']
    status_frame_instance.accel_y = link.rx_obj(obj_type='f', start_pos=rx_size)
    rx_size += serial_transfer.STRUCT_FORMAT_LENGTHS['f']
    status_frame_instance.accel_z = link.rx_obj(obj_type='f', start_pos=rx_size)
    rx_size += serial_transfer.STRUCT_FORMAT_LENGTHS['f']
    status_frame_instance.gyro_x = link.rx_obj(obj_type='f', start_pos=rx_size)
    rx_size += serial_transfer.STRUCT_FORMAT_LENGTHS['f']
    status_frame_instance.gyro_y = link.rx_obj(obj_type='f', start_pos=rx_size)
    rx_size += serial_transfer.STRUCT_FORMAT_LENGTHS['f']
    status_frame_instance.gyro_z = link.rx_obj(obj_type='f', start_pos=rx_size)
    rx_size += serial_transfer.STRUCT_FORMAT_LENGTHS['f']
    status_frame_instance.mag_x = link.rx_obj(obj_type='f', start_pos=rx_size)
    rx_size += serial_transfer.STRUCT_FORMAT_LENGTHS['f']
    status_frame_instance.mag_y = link.rx_obj(obj_type='f', start_pos=rx_size)
    rx_size += serial_transfer.STRUCT_FORMAT_LENGTHS['f']
    status_frame_instance.mag_z = link.rx_obj(obj_type='f', start_pos=rx_size)
    rx_size += serial_transfer.STRUCT_FORMAT_LENGTHS['f']
    status_frame_instance.lsm9_tstamp = link.rx_obj(obj_type='i', start_pos=rx_size)
    rx_size += serial_transfer.STRUCT_FORMAT_LENGTHS['i']

    status_frame_instance.sharpir1 = link.rx_obj(obj_type='f', start_pos=rx_size)
    rx_size += serial_transfer.STRUCT_FORMAT_LENGTHS['f']
    status_frame_instance.sharpir2 = link.rx_obj(obj_type='f', start_pos=rx_size)
    rx_size += serial_transfer.STRUCT_FORMAT_LENGTHS['f']
    status_frame_instance.sharpir3 = link.rx_obj(obj_type='f', start_pos=rx_size)
    rx_size += serial_transfer.STRUCT_FORMAT_LENGTHS['f']
    status_frame_instance.sharpir4 = link.rx_obj(obj_type='f', start_pos=rx_size)
    rx_size += serial_transfer.STRUCT_FORMAT_LENGTHS['f']
    status_frame_instance.sharpir5 = link.rx_obj(obj_type='f', start_pos=rx_size)
    rx_size += serial_transfer.STRUCT_FORMAT_LENGTHS['f']
    status_frame_instance.sharpir6 = link.rx_obj(obj_type='f', start_pos=rx_size)
    rx_size += serial_transfer.STRUCT_FORMAT_LENGTHS['f']
    status_frame_instance.sharpir_tstamp = link.rx_obj(obj_type='i', start_pos=rx_size)
    rx_size += serial_transfer.STRUCT_FORMAT_LENGTHS['i']

    status_frame_instance.sonar1 = link.rx_obj(obj_type='f', start_pos=rx_size)
    rx_size += serial_transfer.STRUCT_FORMAT_LENGTHS['f']
    status_frame_instance.sonar2 = link.rx_obj(obj_type='f', start_pos=rx_size)
    rx_size += serial_transfer.STRUCT_FORMAT_LENGTHS['f']
    status_frame_instance.sonar_tstamp = link.rx_obj(obj_type='i', start_pos=rx_size)
    rx_size += serial_transfer.STRUCT_FORMAT_LENGTHS['i']

    status_frame_instance.tacometer_count = link.rx_obj(obj_type='i', start_pos=rx_size)
    rx_size += serial_transfer.STRUCT_FORMAT_LENGTHS['i']
    status_frame_instance.tacometer_tstamp = link.rx_obj(obj_type='i', start_pos=rx_size)
    rx_size += serial_transfer.STRUCT_FORMAT_LENGTHS['i']

    return status_frame_instance


if __name__ == '__main__':
    pub_
