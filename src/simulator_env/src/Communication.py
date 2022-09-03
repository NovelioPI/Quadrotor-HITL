#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from sensor_msgs.msg import NavSatFix
from hector_uav_msgs.msg import MotorPWM
from serial import Serial

from time import sleep, perf_counter
from threading import Thread


class Communication:
    def __init__(self):
        self._imu_data = Imu()
        self._alt_data = Range()
        self._gps_data = NavSatFix()

    def run_node(self, port='/dev/ttyUSB0', baud=57600, rate=100):
        rospy.init_node('Communication', anonymous=True)
        self.rate = rospy.Rate(rate)

        self._init_serial(port, baud)
        self._init_thread()

    def _update_imu(self):
        while not rospy.is_shutdown():
            try:
                self._imu_data = rospy.wait_for_message('/raw_imu', Imu, timeout=1)    

            except:
                pass
                
        self.rate.sleep()

    def _update_alt(self):
        while not rospy.is_shutdown():
            try:
                self._alt_data = rospy.wait_for_message('/sonar_height', Range, timeout=1)
                
            except:
                pass
                
        self.rate.sleep()
    
    def _update_gps(self):
        while not rospy.is_shutdown():
            try:
                self._gps_data = rospy.wait_for_message('/fix', NavSatFix, timeout=1)
                
            except:
                pass
                
        self.rate.sleep()

    def _get_data(self):
        while not rospy.is_shutdown():
            self._data = "{:.8f},{:.8f},{:.8f},{:.8f},{:.8f},{:.8f},{:.8f},{:.8f},{:.8f},{:.8f}\n".format(
                    self._imu_data.angular_velocity.x,
                    self._imu_data.angular_velocity.y,
                    self._imu_data.angular_velocity.z,
                    self._imu_data.linear_acceleration.x,
                    self._imu_data.linear_acceleration.y,
                    self._imu_data.linear_acceleration.z,
                    self._alt_data.range,
                    self._gps_data.latitude,
                    self._gps_data.longitude,
                    self._gps_data.altitude)
            self.rate.sleep()
    
    def _mcu_read(self):
        while not rospy.is_shutdown():
            start = perf_counter()
            try:
                if not self._ser.open:
                    self._init_serial()
                else:
                    data = self._ser.readline()
                    print(data.decode(), end="")
            except:
                pass
            self.rate.sleep()
            # print("read:",perf_counter() - start)
        else:
            return

    def _mcu_write(self):
        while not rospy.is_shutdown():
            start = perf_counter()
            try:
                if not self._ser.open:
                    self._init_serial()
                else:
                    self._ser.write(self._data.encode())
            except:
                pass
            self.rate.sleep()
            # print("write:",perf_counter() - start)
        else:
            return

    def _init_serial(self, port, baud):
        self._ser = Serial(port, baud, timeout=1)
        try:
            self._ser.open()
            self._ser.flush()
        except:
            pass
    
    def _init_thread(self):
        imu_th = Thread(target=self._update_imu)
        alt_th = Thread(target=self._update_alt)
        gps_th = Thread(target=self._update_gps)
        data_th = Thread(target=self._get_data)
        transmit_th = Thread(target=self._mcu_write)
        receive_th = Thread(target=self._mcu_read)
        
        try:
            imu_th.start()
            alt_th.start()
            gps_th.start()
            data_th.start()
            transmit_th.start()
            receive_th.start()
        except:
            pass
    
    def _shutdown(self):
        self._ser.cancel_read()
        self._ser.cancel_write()
        self._ser.close()

    @property
    def imu(self):
        return self._imu_data
    
    @property
    def alt(self):
        return self._alt_data

    @property
    def alt(self):
        return self._gps_data


if __name__ == '__main__':
    com = Communication()

    com.run_node(port='/dev/ttyACM0', baud=115200,rate=100)