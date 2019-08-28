#!/usr/bin/env python
# license removed for brevity
import rospy
import serial
import time
import tf
import re
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import math
imu_msg = Imu()

ser = serial.Serial(
    port='/dev/agv/imu',
    baudrate=115200,
    parity=serial.PARITY_ODD,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.SEVENBITS)

ser.isOpen()

def imu_data():
    imu_pub   = rospy.Publisher("imu_data", Imu, queue_size=50)
    theta_pub = rospy.Publisher("theta_data", String , queue_size = 50)


    rospy.init_node('razor imu', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():

        t = rospy.Time.now()
       
        imu = ser.read_all().replace('\r','')
        rospy.loginfo(' data :' + imu )
        imu = re.findall(r'-?\d+\.?\d*',imu)
        a = len(imu)
        if a==9 :
            
            imu_msg.header.stamp = t
            imu_msg.header.seq += 1
            imu_msg.header.frame_id         = 'imu'
            imu_msg.linear_acceleration.x   = float(imu[0])
            imu_msg.linear_acceleration.y   = float(imu[1])
            imu_msg.linear_acceleration.z   = float(imu[2])
            imu_msg.angular_velocity.x      = float(imu[3])
            imu_msg.angular_velocity.y      = float(imu[4])
            imu_msg.angular_velocity.z      = float(imu[5])
            h = float(imu[8])*-3.141592/180.0
	    quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, h)
            imu_msg.orientation.w           = quaternion[3]
            imu_msg.orientation.x           = quaternion[0]
            imu_msg.orientation.y           = quaternion[1]
            imu_msg.orientation.z           = quaternion[2]
            
		

            theta_pub.publish(str(h))
            imu_pub.publish(imu_msg)  
        rate.sleep()
        
  
if __name__ == '__main__':
    try:
        imu_data()
    except rospy.ROSInterruptException:
        pass
