#!/usr/bin/python3

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int64, String
import random
from itertools import combinations

num_robots = 3
robots = [f"robot{num}" for num in range(1,num_robots+1)]
bomb_duration = 20
bomb_range = 2
thrower_length_advantage = 2.5

class Robot:
    def __init__(self, name):
        self.name = name

        self.pose_sub = rospy.Subscriber(f"/{name}/pose", PoseStamped, self.set_pose)
        self.stop_sub = rospy.Subscriber(f"/{name}/stop", Int64, self.set_stop)
        self.stop_pub = rospy.Publisher(f"/{name}/stop", Int64, queue_size=10)
        self.bomb_sub = rospy.Subscriber(f"/bomb", PoseStamped, self.escape)
        self.bomb_pub = rospy.Publisher(f"/bomb", PoseStamped, queue_size=10)
        self.target_pub = rospy.Publisher(f"/{name}/bomb_target", String, queue_size=10)
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

    def set_stop(self, msg):
        duration = msg.data
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

    def throw_bomb_at(self, target):
        target_msg = String()
        target_msg.data = target.name
        bomb_msg = target.pose_stamped
        self.bomb_pub.publish(bomb_msg)
        self.target_pub.publish(target_msg)

        stop_duration = Int64()
        stop_duration.data = bomb_duration
        self.stop_pub.publish(stop_duration)
        print('it has been published ' + self.name + ' ' + target.name)


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

    def bomb_calculation(self):
        for robot1, robot2 in list(combinations(self.robots, 2)):
            print('ok desu\n')
            if robot1.distance_to(robot2) <= bomb_range + thrower_length_advantage:
                if random.random() > 0.5:
                    robot1.throw_bomb_at(robot2)
                else:
                    robot1.throw_bomb_at(robot2)

    def run(self):
        while not rospy.is_shutdown():
            for follower in self.robots:
                if not follower.blocked:
                    closest = self.get_closest_robot(follower)
                    follower.set_target(closest.pose_stamped)

            self.bomb_calculation()
            self.rate.sleep()



if __name__=="__main__":
    rospy.init_node('path_planner')
    planner = Planner()
    planner.run()