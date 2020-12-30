import rospy
import actionlib

from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class NavCommander():

    def __init__(self):
        rospy.init_node('j2_navcommander')

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        print("wainting for server")
        self.client.wait_for_server()
        print("Found server")

    def sendGoal(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
       
        goal.target_pose.pose.position.x = 4.755
        goal.target_pose.pose.position.y = -0.992
        goal.target_pose.pose.position.z = 0.0
        
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.956
        goal.target_pose.pose.orientation.w = 0.295

        print("Sending goal..")
        self.client.send_goal(goal)
        print("Did send")

def main(args=None):
    navcommander = NavCommander()
    navcommander.sendGoal()

if __name__ == '__main__':
    main()