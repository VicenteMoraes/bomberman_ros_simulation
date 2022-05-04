import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from morse.builder import *

bomb_range = 1
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

        try:
            robot = self.robots[target.data]
        except KeyError:
            return
        scene = bge.logic.getCurrentScene()
        scene.objects[self.name].worldPosition = scene.objects[robot.name].worldPosition
        scene.objects[self.name].worldPosition.z = 0.8
        self.explode_target = robot.pose_stamped
        rospy.Timer(rospy.Duration(bomb_duration), self.explode, oneshot=True)

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
                msg = Bool()
                msg.data = True
                del robot.robots[robot.name]
                robot.kill_pub.publish(msg)
                try:
                    scene.objects[robot.name].endObject()
                except KeyError:
                    continue

                digit = robot.name[-1]
                caster_wheel = "CasterWheel" if digit == '1' else f"CasterWheel.00{int(digit)-1}"
                wheel_l = "Wheel_L" if digit == '1' else f"Wheel_L.00{int(digit)-1}"
                wheel_r = "Wheel_R" if digit == '1' else f"Wheel_R.00{int(digit)-1}"

                scene.objects[caster_wheel].endObject()
                scene.objects[wheel_l].endObject()
                scene.objects[wheel_r].endObject()

