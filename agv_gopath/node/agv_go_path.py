#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped , PoseStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion , quaternion_from_euler
from std_msgs.msg import String
import time

path_a =  ['0.0;0.0;0.01',
           '4.0;0.0;0.01',
           '4.0;1.0;0.01',
           '0.0;1.0;0.01',
           '0.0;0.0;0.01'
          ]

class client:
    def __init__(self):
        self.path_pub = rospy.Publisher('/path/plan', Path, queue_size=1)
        self.nav_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        self.pose_x =0
        self.pose_y =0
        self.point_step = 0
        self.pub = True
        
        

        

    def callback1(self, data):
        #print data
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y 

    def main1(self):
        
        rospy.init_node('Move_goal', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():

            try:             
                x=[]
                y=[]
                theta=[]                
                if  1==1:
                     
                    for i in range (1, len(path_a)+1,1):

                        X,Y,THETA = path_a[i-1].split(';')
                        x.append(X)
                        y.append(Y)
                        theta.append(THETA)
                        pose = PoseStamped()
                        pose.header.frame_id = "map"
                        pose.pose.position.x = float(X)
                        pose.pose.position.y = float(Y)
                        quaternion = quaternion_from_euler(0,0,float(THETA))
                        pose.pose.orientation.x = quaternion[0]
                        pose.pose.orientation.y = quaternion[1]
                        pose.pose.orientation.z = quaternion[2]
                        pose.pose.orientation.w = quaternion[3]
                        pose.header.seq = path.header.seq + 1
                        path.header.seq = pose.header.seq
                        path.header.frame_id = '/map'
                        path.poses.append(pose)

                    
                    self.path_pub.publish(path)                  
                    
                    #print 'Pass to Step_goal' + self.pub               
                    if self.pub :
                        nav_goal.header.frame_id = 'map'
                        
                        nav_goal.pose.position.x = path.poses[self.point_step].pose.position.x
                        nav_goal.pose.position.y = path.poses[self.point_step].pose.position.y
                        nav_goal.pose.orientation = path.poses[self.point_step].pose.orientation     
                        #self.nav_pub.publish(nav_goal)
                        #rospy.sleep(0.1)
                        self.nav_pub.publish(nav_goal)
                        self.pub = False
                    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped , self.callback1 )
                    if abs(self.pose_x - path.poses[self.point_step].pose.position.x) < 0.2 and abs(self.pose_y - path.poses[self.point_step].pose.position.y) < 0.2:
                        self.point_step = self.point_step +1
                        self.pub = True
                        print self.point_step
                    if self.point_step > len(path_a)-1 :
                        self.point_step = 0
                    
                    
                    path.poses =[]        
                    
            except rospy.ROSInterruptException:
                pass
                          

            rate.sleep()
        
if __name__ == '__main__':

    try:
        path = Path() 
        pose = PoseStamped()
        nav_goal = PoseStamped()

        Run = client()
        Run.main1()
    except rospy.ROSInterruptException:       
        pass
