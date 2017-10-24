import time
import roslaunch
import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry

from Misc import bcolors

class LaunchManager():
    def __init__(self,_MaxRestartCount):
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.rLaunchBringupBack = roslaunch.parent.ROSLaunchParent(self.uuid, ["/opt/ros/indigo/share/turtlebot_bringup/launch/minimal.launch"])
        self.rLaunchBringupNav = roslaunch.parent.ROSLaunchParent(self.uuid, ["/opt/ros/indigo/share/turtlebot_bringup/launch/minimal.launch"])
        self.rLaunchBringupReading = roslaunch.parent.ROSLaunchParent(self.uuid, ["/opt/ros/indigo/share/turtlebot_bringup/launch/minimal.launch"])
        self.rLaunchAmclDemo = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/turtlebot/BJ/launch/amcl_demo.launch"])
        #self.rLaunchAmclDemoBack = roslaunch.parent.ROSLaunchParent(self.uuid,["/home/turtlebot/BJ/launch/amcl_demo.launch"])
        self.rLaunchVisp = roslaunch.parent.ROSLaunchParent(self.uuid,["/home/turtlebot/BJ/launch/tracklive_usb.launch"])
        self.rLaunchKobuki = roslaunch.parent.ROSLaunchParent(self.uuid,["/home/turtlebot/BJ/launch/compact.launch"])
        self.rLaunchDocking = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/turtlebot/BJ/launch/activate.launch"])
        self.rLaunchSoundplay = roslaunch.parent.ROSLaunchParent(self.uuid, ["/home/turtlebot/BJ/launch/soundplay_node.launch"])
        #self.MaxRestartCount=_MaxRestartCount
        #self.CurrentRestart=0
        #self.CreateRestartArray()

    def CreateRestartArray(self):
         self.rLaunchAmclDemoBack=[]
         for i in range(1,self.MaxRestartCount):
            self.rLaunchAmclDemoBack.append(roslaunch.parent.ROSLaunchParent(self.uuid,["/home/turtlebot/BJ/launch/amcl_demo.launch"]))
            rospy.sleep(1)

    def StartBackLaunchFile(self):
        print(bcolors.OKBLUE + "Launch Bringup for Back" + bcolors.ENDC)
        self.rLaunchBringupBack.start()
        self.WaitForRoscoreStart()
        rospy.init_node('TurtleBotController', anonymous=False)

    def TerminateBack(self):
        print(bcolors.OKBLUE + "Terminate Back" + bcolors.ENDC)
        self.rLaunchBringupBack.shutdown()

    def StartNavLaunchFile(self):
        print(bcolors.OKBLUE + "Launch Bringup for Nav" + bcolors.ENDC)
        self.rLaunchBringupNav.start()
        self.WaitForRoscoreStart()
        print(bcolors.OKBLUE + "Launch AmclDemo for Nav" + bcolors.ENDC)
        #self.rLaunchAmclDemoBack[self.CurrentRestart].start()
        self.rLaunchAmclDemo.start()
        print(bcolors.OKBLUE + "Launch Visp for Nav" + bcolors.ENDC)
        self.rLaunchVisp.start()
        self.WaitForRoscoreStart()
        rospy.init_node('TurtleBotController', anonymous=False)

    def RestartAmcl(self):
        #self.rLaunchAmclDemoBack[self.CurrentRestart].shutdown()
        self.rLaunchAmclDemo.shutdown()
        rospy.sleep(5)
        self.rLaunchAmclDemo = roslaunch.parent.ROSLaunchParent(self.uuid,
                                                                ["/home/turtlebot/BJ/launch/amcl_demo.launch"])
        rospy.sleep(5)
        self.rLaunchAmclDemo.start()
        return True

    def TerminateNav(self):
        print(bcolors.OKBLUE + "Terminate Nav" + bcolors.ENDC)
        self.rLaunchAmclDemo.shutdown()
        self.rLaunchVisp.shutdown()
        self.rLaunchBringupNav.shutdown()
        self.WaitForRoscoreClose()

    def StartDockingLaunchFile(self):
        print(bcolors.OKBLUE + "Launch Kobuki Base" + bcolors.ENDC)
        self.rLaunchKobuki.start()
        print(bcolors.OKBLUE + "Launch Docking" + bcolors.ENDC)
        self.rLaunchDocking.start()
        self.WaitForRoscoreStart();
        rospy.init_node('TurtleBotController', anonymous=False)

    def TerminateDocking(self):
        print(bcolors.OKBLUE + "Terminate Docking" + bcolors.ENDC)
        self.rLaunchDocking.shutdown()
        self.rLaunchKobuki.shutdown()
        self.WaitForRoscoreClose()

    def StartTtsLaunchFile(self):
        print(bcolors.OKBLUE + "Launch Bringup" + bcolors.ENDC)
        self.rLaunchBringupReading.start()
        self.WaitForRoscoreStart()
        print(bcolors.OKBLUE + "Launch Soundplay" + bcolors.ENDC)
        self.rLaunchSoundplay.start()
        self.WaitForRoscoreStart()
        rospy.init_node('TurtleBotController', anonymous=False)

    def TerminateTts(self):
        self.rLaunchSoundplay.shutdown()
        self.rLaunchBringupNav.shutdown()
        self.WaitForRoscoreClose()

    def WaitForRoscoreStart(self):
        bRoscoreFail = True;
        print(bcolors.OKBLUE + "===========Waiting For Roscore=========" + bcolors.ENDC)
        while bRoscoreFail == True:
            bRoscoreFail = False
            try:
                rospy.get_master().getPid()
            except:
                bRoscoreFail = True
            rospy.sleep(1)
        print(bcolors.OKBLUE + "===========Roscore Running=========" + bcolors.ENDC)

    def WaitForRoscoreClose(self):
        bRoscoreRunning = True;
        print(bcolors.OKBLUE + "===========Waiting For Roscore to Close=========" + bcolors.ENDC)
        while bRoscoreRunning == True:
            try:
                rospy.get_master().getPid()
            except:
                bRoscoreRunning = False
            rospy.sleep(1)
        print(bcolors.OKBLUE + "===========Roscore Closed=========" + bcolors.ENDC)






