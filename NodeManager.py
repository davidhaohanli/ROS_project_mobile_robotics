import time
import roslaunch
import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry

from Misc import bcolors

class NodeManager():
    def __init__(self):
        rospy.on_shutdown(self.shutdown)
        self.current_pose = PoseWithCovarianceStamped()
        self.current_qr = String()
        self.current_odom = Odometry()


    def update_current_odom(self, _current_odom):
        self.current_odom = _current_odom

    def update_current_pose(self, _current_pose):
        self.current_pose = _current_pose

    def update_current_qr(self, _current_qr):
        if _current_qr.data!="":
            self.current_qr = _current_qr.data


    def WaitForNavNodes(self):
        self.WaitForOdom()
        self.WaitForAmcl()

    def WaitForOdom(self):
        print(bcolors.OKBLUE + "===========Waiting For Nodes=========" + bcolors.ENDC)
        rospy.wait_for_message('odom', Odometry)
        print(bcolors.OKBLUE + "===========First Odometry Data Received=========" + bcolors.ENDC)
        self.amclSubscriberHandle=rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.update_current_pose)
        self.code_messageSubscriberHandle =rospy.Subscriber('/visp_auto_tracker/code_message', String, self.update_current_qr)
        self.odomSubscriberHandle =rospy.Subscriber('odom', Odometry, self.update_current_odom)
        print(bcolors.OKBLUE + "===========Subscribers Called=========" + bcolors.ENDC)
        while(self.current_odom.header.seq<=1):
            rospy.sleep(1)
        print(bcolors.OKBLUE + "===========Good Odometry Data Received=========" + bcolors.ENDC)

    def WaitForAmcl(self):
        while abs(self.current_pose.pose.pose.position.x) < 1E-9:
            rospy.sleep(1)
            print(bcolors.OKBLUE + "Waiting Amcl" + bcolors.ENDC)
            print(self.current_pose.pose.pose.position.x)
        self.amclSubscriberHandle.unregister()
        self.odomSubscriberHandle.unregister()


    def shutdown(self):
        rospy.loginfo("Stopping the nodes...")

