import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
import math


class NavController:
    _navmatrix = [
            [[7, -7], [3, -7], [-2, -7], [-7, 7]],
            [[7, -3], [3, -3], [-2, -3], [-7, -3]],
            [[7, 2], [3, 2], [-2, 2], [-7, 2]],
            [[7, 6], [3, 6], [-2, 6], [-7, 6]]]

    # Matrix positions for each possible y coordinate
    _ydict = {-7: 0, -3: 1, 2: 2, 6:3}

    # Matrix positions for each possible x coordinate
    _xdict = {7: 0, 3: 1, -2: 2, -7:3}

    def __init__(self, robot, precision=0.1):
        self.robot = robot
        self.velocity_pub = rospy.Publisher(f"{self.robot.name}/cmd_vel", Twist, 10)
        self.pub = rospy.Publisher("help", String, 10)
        self.precision = precision
        self.speed = 1
        self.turning_speed = 1
        self.target = [-2, -3]
        self.pose = None
        self.sub = rospy.Subscriber(f"{self.robot.name}/pose", PoseStamped, self.update)

    def get_matrix_position_from_target(self):
        pass

    def get_x(self, obj):
        if type(obj) == PoseStamped:
            return obj.pose.position.x
        else:
            return obj[0]

    def send(self, pp):
        s = String()
        s.data = pp
        self.pub.publish(s)

    def get_y(self, obj):
        if type(obj) == PoseStamped:
            return obj.pose.position.y
        else:
            return obj[1]

    def distance(self, pose, target):
        return ((self.get_x(pose) - self.get_x(target)) ** 2 + (self.get_x(pose) - self.get_x(target)) ** 2) ** 0.5

    def pose_equals_target(self, pose, target):
        return self.get_y(pose) - self.precision <= self.get_y(target) <= self.get_y(pose) + self.precision and self.get_x(pose) - self.precision <= self.get_x(target) <= self.get_x(pose) + self.precision 

    def update(self, pose):
        self.pose = pose
        twist = Twist()
        if self.pose_equals_target(self.pose, self.target):
            return
        if self.get_y(self.pose) < self.get_y(self.target):
            if math.pi / 2 - self.precision <= self.pose.pose.orientation.z <= math.pi / 2 + self.precision:
                twist.linear.x = self.speed
                twist.angular.z = 0
            else:
                twist.linear.x = 0
                twist.angular.z = self.turning_speed
        elif self.get_y(self.pose) > self.get_y(self.target):
            if -math.pi / 2 - self.precision <= self.pose.pose.orientation.z <= -math.pi / 2 + self.precision:
                twist.linear.x = -self.speed
                twist.angular.z = 0
            else:
                twist.linear.x = 0
                twist.angular.z = self.turning_speed
        else:
            if self.get_x(self.pose) > self.get_x(self.target):
                twist.linear.x = self.speed
            else:
                twist.linear.x = -self.speed
            twist.angular.z = 0

        self.velocity_pub.publish(twist)

