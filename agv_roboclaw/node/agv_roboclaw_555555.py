#!/usr/bin/env python
# license removed for brevity
import rospy
import time
import tf
from nav_msgs.msg import Odometry
from roboclaw import Roboclaw
from geometry_msgs.msg import Twist, Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import String
import math
odom = Odometry()
#TICK2RAD   = 0.012566371
TICK2Mater = 0.000199491
WHEEL_RADIUS = 0.127/2.0
WHEEL_SEPARATION =0.355

odom_pose = [0.0, 0.0, 0.0]
wheel_velocity_cmd_lift = 0
wheel_velocity_cmd_right = 0
wheel_separation = 0.355
rc = Roboclaw("/dev/agv/roboclaw",115200)

address = 0x80
def callback(data):
    global wheel_velocity_cmd_lift 
    wheel_velocity_cmd_lift = float(data.linear.x -(data.angular.z*wheel_separation*0.5))*127.0
    global wheel_velocity_cmd_right 
    wheel_velocity_cmd_right = float(data.linear.x +(data.angular.z*wheel_separation*0.5))*127.0


def cmd_vel_move():
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    rospy.init_node('agv_roboclaw', anonymous=True)
    count_r = 0
    count_l = 0
    rate = rospy.Rate(10) # 10hz
    rc.Open()
    
    while not rospy.is_shutdown():
        rospy.Subscriber("/cmd_vel", Twist, callback)
        en_l = rc.ReadEncM1(address)
        en_r = rc.ReadEncM2(address)
        time.sleep(0.005)
        rc.ResetEncoders(address)
        wheel_l = TICK2Mater * en_l[1]
        wheel_r = TICK2Mater * en_r[1]
        
        if math.isnan(wheel_l):
            wheel_l = 0.0 
        if math.isnan(wheel_r):
            wheel_r = 0.0
        #drive Motor
        vl = int(wheel_velocity_cmd_lift)
        vr = int(wheel_velocity_cmd_right)
        
        if vl >= 0:
            rc.ForwardM1(0x80,vl)
        else:
            rc.BackwardM1(0x80,abs(vl))
        if vr >= 0:
            rc.ForwardM2(0x80,vr)
        else:
            rc.BackwardM2(0x80,abs(vr))
     

        # if wheel_velocity_cmd_right > 0 : 
        #     rc.ForwardM2(0x80, int(wheel_velocity_cmd_right))
        # if wheel_velocity_cmd_right < 0 :
        #     rc.BackwardM2(0x80, abs(int(wheel_velocity_cmd_right))
 

        delta_s = (wheel_r + wheel_l) / 2.0
        theta = (wheel_r - wheel_l) / WHEEL_SEPARATION
	
        odom_pose[1] += delta_s * math.sin(odom_pose[2] + (theta / 2.0))
        odom_pose[0] += delta_s * math.cos(odom_pose[2] + (theta / 2.0))
        odom_pose[2] += theta
        rospy.loginfo(' wheel_velocity_cmd_lift :' + str(wheel_velocity_cmd_lift) +' wheel_velocity_cmd_right :'   + str(wheel_velocity_cmd_right) )

        rospy.loginfo(' wheel_L :' + str(wheel_l) +' wheel_R :'   + str(wheel_r) )
        current_time = rospy.Time.now()
        odom.header.stamp = current_time


        br = tf.TransformBroadcaster()
        br.sendTransform(( odom_pose[0], odom_pose[1], 0),
                         tf.transformations.quaternion_from_euler(0, 0, odom_pose[2]),
                         current_time,
                         "base_footprint",
                         "odom")
        odom.header.frame_id = "odom"
        odom.child_frame_id  = 'base_footprint'
        odom.pose.pose.position.x = odom_pose[0]
        odom.pose.pose.position.y = odom_pose[1]
        odom.pose.pose.position.z = 0
      
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, odom_pose[2])
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        odom.pose.covariance[0] = 0.1
        odom.pose.covariance[7] = 0.1
        odom.pose.covariance[14] = 1e6
        odom.pose.covariance[21] = 1e6
        odom.pose.covariance[28] = 1e6
        odom.pose.covariance[35] = 1e6

        odom.twist.covariance[0] = 0.0001
        odom.twist.covariance[7] = 0.0001 
        odom.twist.covariance[14] = 1e6
        odom.twist.covariance[21] = 1e6
        odom.twist.covariance[28] = 1e6
        odom.twist.covariance[35] = 0.0001      
        odom_pub.publish(odom)
      
        rate.sleep()    
    print ('stop55')
    rc.ForwardM1(0x80, 0)

if __name__ == '__main__':
    try:
        cmd_vel_move()
     
    except rospy.ROSInterruptException:
        pass
