import rospy
from geometry_msgs.msg import PoseStamped
from morse.builder import *

bomb_range = 2
bomb_duration = 10

class Bomb:
    def __init__(self, robot, name):
        self.bomb = PassiveObject('/bomberman_ws/bomberman/map/bomb.blend', name)
        self.bomb.translate(x=robot.x, y=robot.y, z=-10)
        self.bomb_pub = rospy.Publisher("bomb", PoseStamped, queue_size=1)
        
        self.name = name
        self.robot = robot
        self.robots = robot.robots

    def launch(self, target):
        try:
            import bge
        except ImportError:
            # Game engine hasn't started yet
            return

        robot = self.robots[target.data]
        scene = bge.logic.getCurrentScene()
        scene.objects[self.name].worldPosition = scene.objects[robot.name].worldPosition
        self.explode_target = robot.pose_stamped
        rospy.Timer(rospy.Duration(bomb_duration), self.explode)

        self.bomb_pub.publish(robot.pose_stamped)

    def explode(self, _):
        try:
            import bge
        except ImportError:
            # Game engine hasn't started yet
            return

        scene = bge.logic.getCurrentScene()
        scene.objects[self.name].worldPosition.z = -10
        for robot in list(self.robots.values()):
            if robot.distance_to(self.explode_target) < bomb_range:
                scene.objects[robot.name].endObject()

