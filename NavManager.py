import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import Twist

import tf
#from transform_utils import quat_to_angle, normalize_angle
from math import pow, pi, fabs

import csv
from Misc import bcolors
from GoToPose import *
from RMath import *

class NavManager():
    def __init__(self,_path,_initialCount):
        self.path = _path;
        rospy.on_shutdown(self.shutdown)
        self.actualPosition = Point()
        self.actualQuaternion = Quaternion()
        self.actualPositionBackup = Point()
        self.actualQuaternionBackup = Quaternion()
        self.lWayPointList=list()
        self.Counter = _initialCount
        self.CounterBackup=0;
        self.TotalWayPoints = 0
        self.ReadWayPoints()
        self.dockingStatus=-1;
        self.InitialPosition = Point()
        self.InitialQuaternion = Quaternion()
        self.bGotoInitial=False

    def BackupActualPose(self):
        rospy.wait_for_message('amcl_pose', PoseWithCovarianceStamped)
        self.actualPositionBackup= self.actualPosition
        self.actualQuaternionBackup = self.actualQuaternion
        self.CounterBackup=self.Counter

    def IfTripFinished(self):
        if(self.Counter >= self.TotalWayPoints and self.bGotoInitial == True):
            return True
        else:
            return False

    def IfDockingFinished(self):
        if self.dockingStatus==3:
            return True
        else:
            return False

    def ManageBackoff(self):
        self.cmd_vel = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=5)
        self.tf_listener = tf.TransformListener()
        rospy.sleep(2)
        self.odom_frame = '/odom'
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")

    def UnmanageBackoff(self):
        return

    def ManageNav(self):
        self.navigator = GoToPose()
        self.amclSubscriberHandle = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped, self.update_current_pose)
        self.PoseWithCovarianceStamped_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=5)

    def UnmanageNav(self):
        self.amclSubscriberHandle.unregister();

    def update_current_pose(self, _current_pose):
        self.actualPosition = _current_pose.pose.pose.position
        self.actualQuaternion = _current_pose.pose.pose.orientation

    def MonitorDocking(self):
        self.dockingStatusSubscriberHandle = rospy.Subscriber('/dock_drive_action/status', GoalStatusArray, self.update_docking_status)
        rospy.wait_for_message('/dock_drive_action/status', GoalStatusArray)

    def UnmonitorDocking(self):
        self.dockingStatusSubscriberHandle.unregister();

    def update_docking_status(self, _GoalStatusArray):
        if(_GoalStatusArray.status_list!=[]):
            if (_GoalStatusArray.status_list[0] != []):
                self.dockingStatus =_GoalStatusArray.status_list[0].status

    def ReadWayPoints(self):
        file = open(self.path, "r")
        reader = csv.reader(file)
        self.lWayPointList=list(reader)
        print(bcolors.OKGREEN + "==================Waypoints Loaded==========================" + bcolors.ENDC)
        print(self.lWayPointList)
        print(bcolors.OKGREEN + "============================================================" + bcolors.ENDC)
        self.TotalWayPoints=len(self.lWayPointList)

    def TraverseNext(self):
        if(self.Counter<self.TotalWayPoints):
            # Customize the following values so they are appropriate for your location
            Coord=PixelCoorToPQ(float(self.lWayPointList[self.Counter][0]),\
                                float(self.lWayPointList[self.Counter][1]),\
                                float(self.lWayPointList[self.Counter][2]))
            position = {'x': Coord[0],'y': Coord[1]}
            quaternion = {'r1': 0,'r2': 0,'r3': Coord[2],'r4': Coord[3]}
            rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
            success = self.navigator.goto(position, quaternion)
            if success:
                rospy.loginfo("Hooray, reached the desired pose")
                rospy.sleep(1)
                self.Counter = self.Counter + 1
                #print("Traverse Waypoint Counter:")
                #print(self.Counter)
                return True
            else:
                rospy.loginfo("The base failed to reach the desired pose")
                rospy.sleep(1)
                return False
            # Sleep to give the last log messages time to be sent


        else:
            if(self.bGotoInitial==False):
                position = {'x': self.InitialPosition.x, 'y': self.InitialPosition.y}
                quaternion = {'r1': 0, 'r2': 0, 'r3': self.InitialQuaternion.z, 'r4': self.InitialQuaternion.w}
                success = self.navigator.goto(position, quaternion)
                if success:
                    rospy.loginfo("The robot is home")
                    self.bGotoInitial = True
                    return True
                else:
                    rospy.loginfo("The robot is permanantly lost!!!")
            return True
            #rospy.logwarn("NavManager.TraverseNext():Depleted all waypoints, yet function keeps getting called")

    def SetInitialPosQ(self, point,quaternion,bHighVar):
        start_pos = PoseWithCovarianceStamped()
        # filling header with relevant information
        start_pos.header.frame_id = "map"
        start_pos.header.stamp = rospy.Time.now()
        # filling payload with relevant information gathered from subscribing
        # to initialpose topic published by RVIZ via rostopic echo initialpose
        start_pos.pose.pose.position = point
        start_pos.pose.pose.orientation = quaternion
        '''
        start_pos.pose.pose.position.x = x  # 43#-2.73679924011
        start_pos.pose.pose.position.y = y  # 10.9#-8.84320449829
        start_pos.pose.pose.position.z = 0.0

        start_pos.pose.pose.orientation.x = 0.0
        start_pos.pose.pose.orientation.y = 0.0
        start_pos.pose.pose.orientation.z = sin(radians(theta * 0.5))  # -8.84320449829
        start_pos.pose.pose.orientation.w = cos(radians(theta * 0.5))  # 0.689504217398

        if (start_pos.pose.pose.orientation.w < 0):
            start_pos.pose.pose.orientation.w = -start_pos.pose.pose.orientation.w;
            start_pos.pose.pose.orientation.z = -start_pos.pose.pose.orientation.z;
        '''
	if(bHighVar==True):
            start_pos.pose.covariance = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, \
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
	else:
	    start_pos.pose.covariance = [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, \
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, \
                                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01]

        ##############Check if successful########################
        i = 10
        dist = 10000;
        while i > 0:
            i = i - 1;
            self.PoseWithCovarianceStamped_pub.publish(start_pos)
            rospy.wait_for_message('amcl_pose', PoseWithCovarianceStamped)
            dist = PointDist(start_pos.pose.pose.position, self.actualPosition)
            if (dist < 0.2):
                break;
            rospy.sleep(1)
        if i <= 1:
            rospy.logerr("NavManager.SetInitialPos():Fail to set initial position")
            #############################################################################

    def SetInitialPosCoord(self, x, y, theta, bPixel):
        [PtX, PtY, r3, r4]=PixelCoorToPQ(x, y, theta)
        if (bPixel == False):
            PtX = x
            PtY = y
        self.SetInitialPosQ(Point(PtX,PtY,0),Quaternion(0,0,r3,r4),True)

    def shutdown(self):
        rospy.loginfo("Stopping the Navigation...")
        self.cmd_vel.publish(Twist())

    def Backoff(self,goal_distance):
        rate = 20
        r = rospy.Rate(rate)
        linear_speed = -0.2
        move_cmd = Twist()
        # Set the movement command to forward motion
        move_cmd.linear.x = linear_speed
        # Get the starting position values
        position=Point()
        (position,rotation) =self.get_odom()
        #print(self.get_odom())
        x_start = position.x
        y_start = position.y
        distance = 0
        while distance < goal_distance and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle
            self.cmd_vel.publish(move_cmd)
            r.sleep()
            # Get the current position
            (position,rotation) = self.get_odom()
            #self.get_odom()
            # Compute the Euclidean distance from the start
            distance = sqrt(pow((position.x - x_start), 2) +
                            pow((position.y - y_start), 2))
        #Stop the robot
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def Rotate(self,goal_angle):
        rate = 20
        r = rospy.Rate(rate)
        #goal_angle = pi*2.5
        angular_speed = 0.5
        angular_tolerance = radians(2.5)

        (position, rotation) = self.get_odom()
        last_angle=rotation
        move_cmd = Twist()
        move_cmd.angular.z = angular_speed
        turn_angle = 0
        while abs(turn_angle + angular_tolerance) < abs(goal_angle) and not rospy.is_shutdown():
            # Publish the Twist message and sleep 1 cycle
            self.cmd_vel.publish(move_cmd)
            r.sleep()
            # Get the current rotation
            (position, rotation) = self.get_odom()
            # Compute the amount of rotation since the last loop
            angle1=QuaternionToAngle(rotation);
            angle2=QuaternionToAngle(last_angle)
            delta_angle = Wrap2Pi(QuaternionToAngle(rotation) - QuaternionToAngle(last_angle))
            # Add to the running total
            turn_angle += fabs(delta_angle)
            last_angle = rotation
            # Stop the robot before the next leg
        #Stop the robot
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        #return (Point(*trans), quat_to_angle(Quaternion(*rot)))
        return (Point(trans[0], trans[1], trans[2]), Quaternion(rot[0], rot[1], rot[2], rot[3]))




