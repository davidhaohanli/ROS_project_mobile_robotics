import rospy

from Misc import bcolors
from LaunchManager import LaunchManager
from NodeManager import NodeManager
from NavManager import NavManager
from SpeechManager import SpeechManager
from math import pi
from geometry_msgs.msg import PoseWithCovarianceStamped

if __name__ == '__main__':
    cSpeechManager = SpeechManager()
    cLaunchManager=LaunchManager(10)
    cNodeManager = NodeManager()
    cNavManager = NavManager('/home/turtlebot/BJ/actual_waypoint_list.csv',0)
    print(bcolors.OKBLUE + "======================All Classes Initialized========================" + bcolors.ENDC)
    print(bcolors.OKBLUE + "=====================================================================" + bcolors.ENDC)
    if 1:
        print(bcolors.OKBLUE + "=======================Prepare Navigation========================================" + bcolors.ENDC)
        cLaunchManager.StartNavLaunchFile()
        cNodeManager.WaitForNavNodes()
        cNavManager.ManageNav()
        cSpeechManager.InitializeQrRecording()
        cNavManager.SetInitialPosCoord(826, 70, 0, True)
	print("Initial Start Setting:")
        print(cNavManager.actualPosition)
        if 1:
            cNavManager.ManageBackoff()
            cNavManager.Backoff(1.05)
            cNavManager.Rotate(pi*2.5)
            cNavManager.UnmanageBackoff()
 	    cNavManager.InitialPosition = cNavManager.actualPosition
            cNavManager.InitialQuaternion = cNavManager.actualQuaternion
	    print("Initial Start Setting:")
 	    print(cNavManager.InitialPosition)	
            print(cNavManager.InitialQuaternion)
            print(bcolors.OKGREEN + "========================DONE==========================" + bcolors.ENDC)
            cNavManager.SetInitialPosQ(cNavManager.InitialPosition,cNavManager.InitialQuaternion,False)
           
        while 1:
            rospy.sleep(1)
            print(bcolors.OKGREEN + "========================Navigating==========================" + bcolors.ENDC)
            print(cNavManager.actualPosition)
            print(cNavManager.actualQuaternion)
            print("Core Waypoint Counter:")
            print(cNavManager.Counter)
            print(cNavManager.TotalWayPoints)
            #cNavManager.TraverseNext()
            if(cNavManager.TraverseNext()==False and cNavManager.IfTripFinished()==False):
                print(bcolors.OKGREEN + "========================UnmanageNav==========================" + bcolors.ENDC)
                cNavManager.UnmanageNav();
                print(bcolors.OKGREEN + "========================BackupActualPose==========================" + bcolors.ENDC)
                cNavManager.BackupActualPose();
                print(bcolors.OKGREEN + "========================Restart AMCL==========================" + bcolors.ENDC)
                if(cLaunchManager.RestartAmcl()==False):
                    print(bcolors.WARNING + "========================AMCL Seed Exhausted==========================" + bcolors.ENDC)
                    exit();
                print(bcolors.OKGREEN + "========================Waiting For Nodes==========================" + bcolors.ENDC)
                rospy.sleep(10);
                cNodeManager.WaitForNavNodes();
                print(bcolors.OKGREEN + "=====================Restart Navigating===================" + bcolors.ENDC)
                print("The Last Saved position is")
                print(cNavManager.actualPositionBackup)
                print(cNavManager.actualQuaternionBackup)
                cNavManager.ManageNav()
                cNavManager.SetInitialPosQ(cNavManager.actualPositionBackup,cNavManager.actualQuaternionBackup,True)
                #cNavManager.Counter=cNavManager.CounterBackup;

            if(cNavManager.IfTripFinished()==True):
                break
            #cNavManager.Counter=cNavManager.Counter+1
            print("Core Waypoint Counter at Tne of Loop:")
            print(cNavManager.Counter)
            print(cNavManager.TotalWayPoints)
        print(bcolors.OKGREEN + "=================The robot is back! =====================" + bcolors.ENDC)
        cNavManager.UnmanageNav()
        cSpeechManager.TerminateQrRecording()
        cLaunchManager.TerminateNav()
    #############################################################################################################
    if 1:
        print(bcolors.OKGREEN + "======================Start Docking===============" + bcolors.ENDC)
        cLaunchManager.StartDockingLaunchFile()
        cNavManager.MonitorDocking()
        while cNavManager.IfDockingFinished()== False:
            rospy.sleep(1.)
        cNavManager.UnmonitorDocking()
        cLaunchManager.TerminateDocking()
        print(bcolors.OKGREEN + "===============The robot is docked!===============" + bcolors.ENDC)
    #############################################################################################################
    if 1:

        print(bcolors.OKGREEN + "==========================Start Reading=======================" + bcolors.ENDC)
        cLaunchManager.StartTtsLaunchFile()
        cSpeechManager.InitializeTTS()
        cSpeechManager.RemoveRepeats()
        cSpeechManager.ReadWords()
        cLaunchManager.TerminateTts()
        print(bcolors.OKGREEN + "====================All Tasks Finished===============" + bcolors.ENDC)



