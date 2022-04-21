#!/usr/bin/python3

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int64

num_robots = 3
robots = [f"robot{num}" for num in range(1,num_robots+1)]
bomb_duration = 5
bomb_range = 2

class Robot:
    def __init__(self, name):
        self.name = name

        self.pose_sub = rospy.Subscriber(f"/{name}/pose", PoseStamped, self.set_pose)
        self.stop_sub = rospy.Subscriber(f"/{name}/stop", Int64, self.set_stop)
        self.bomb_sub = rospy.Subscriber(f"/bomb", PoseStamped, self.escape)
        self.movebase_client = actionlib.SimpleActionClient(f"{name}/move_base", MoveBaseAction)

        self.blocked = False
        self.pose_stamped = PoseStamped()

    def distance_to(self, other):
        # Euclidean distance
        a = self.pose_stamped
        try:
            b = other.pose_stamped
        except AttributeError:
            b = other
        return ((a.pose.position.x - b.pose.position.x)**2 + (a.pose.position.y - b.pose.position.y)**2) ** 0.5

    def reset_movement(self, _):
        self.blocked = False

    def set_pose(self, pose):
        self.pose_stamped = pose

    def set_stop(self, duration):
        self.stop()
        self.blocked = True
        rospy.Timer(rospy.Duration(duration), self.reset_movement)

    def set_target(self, target_pose):
        now = rospy.Time.now()
        self.movebase_client.cancel_goals_at_and_before_time(now)
        self.movebase_client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose = target_pose
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = now
        self.movebase_client.send_goal(goal)

    def stop(self):
        self.movebase_client.cancel_goals_at_and_before_time(rospy.Time.now())

    def escape(self, bomb_pose):
        if self.distance_to(bomb_pose) <= bomb_range:
            self.blocked = True
            new_target = self.pose_stamped
            new_target.pose.position.y += min(self.pose_stamped.pose.position.y + 3, 6)
            self.set_target(new_target)
            rospy.Timer(rospy.Duration(bomb_duration), self.reset_movement)


class Planner:
    def __init__(self):
        self.robots = [Robot(robot) for robot in robots]
        self.rate = rospy.Rate(0.1)

    def get_closest_robot(self, robot):
        closest = (None, float('inf'))
        for neighbour in self.robots:
            if robot == neighbour:
                continue
            distance = robot.distance_to(neighbour)
            if distance  < closest[1]:
                closest = (neighbour, distance)
        return closest[0]

    def run(self):
        while not rospy.is_shutdown():
            for follower in self.robots:
                if not follower.blocked:
                    closest = self.get_closest_robot(follower)
                    follower.set_target(closest.pose_stamped)
            self.rate.sleep()



if __name__=="__main__":
    rospy.init_node('path_planner')
    planner = Planner()
    planner.run()
