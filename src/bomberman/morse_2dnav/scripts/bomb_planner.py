#!/usr/bin/python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int64, String
import random
from itertools import combinations

num_robots = 4
robots = [f"robot{num}" for num in range(1,num_robots+1)]
bomb_duration = 10
bomb_range = 2
thrower_length_advantage = 1.5

class Robot:
    def __init__(self, name):
        self.name = name

        self.pose_sub = rospy.Subscriber(f"/{name}/pose", PoseStamped, self.set_pose)
        self.stop_pub = rospy.Publisher(f"/{name}/stop", Int64, queue_size=1)
        self.target_pub = rospy.Publisher(f"/{name}/bomb_target", String, queue_size=1)
        self.pose_stamped = PoseStamped()
        self.bomb_available = True

    def reset_bomb(self, _):
        self.bomb_available = True

    def distance_to(self, other):
        # Euclidean distance
        a = self.pose_stamped
        try:
            b = other.pose_stamped
        except AttributeError:
            b = other
        return ((a.pose.position.x - b.pose.position.x)**2 + (a.pose.position.y - b.pose.position.y)**2) ** 0.5

    def set_pose(self, pose):
        self.pose_stamped = pose

    def throw_bomb_at(self, target):
        target_msg = String()
        target_msg.data = target.name
        self.target_pub.publish(target_msg)

        stop_duration = Int64()
        stop_duration.data = bomb_duration
        self.stop_pub.publish(stop_duration)

        self.bomb_available = False
        rospy.Timer(rospy.Duration(bomb_duration), self.reset_bomb)


class Planner:
    def __init__(self):
        self.robots = [Robot(robot) for robot in robots]
        self.rate = rospy.Rate(1)

    def bomb_calculation(self):
        for robot1, robot2 in list(combinations(self.robots, 2)):
            if robot1.bomb_available and robot1.distance_to(robot2) <= bomb_range + thrower_length_advantage:
                if random.random() < 0.5:
                    robot1.throw_bomb_at(robot2)
                else:
                    robot2.throw_bomb_at(robot1)

    def run(self):
        while not rospy.is_shutdown():
            self.bomb_calculation()
            self.rate.sleep()



if __name__=="__main__":
    rospy.init_node('path_planner')
    planner = Planner()
    planner.run()
