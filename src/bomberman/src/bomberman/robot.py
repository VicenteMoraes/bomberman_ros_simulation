import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from morse.builder import *
import random

bomb_range = 2
bomb_duration = 20

class PioneerRobot(Pioneer3DX):
    number_of_robots = 3
    robots = {}

    def __init__(self, name: str = ""):
        Pioneer3DX.__init__(self, name)
        self.name = name
        self.pose_stamped = PoseStamped()
        self.pose_sub = rospy.Subscriber(f"/{self.name}/pose", PoseStamped, self.set_pose)
        self.bomb_pub = rospy.Publisher("bomb", PoseStamped, queue_size=10)
        self.bomb_target_sub = rospy.Subscriber(f"/{self.name}/bomb_target", String, self.launch_bomb)
        self.bomb_name = "bomb" if name == "robot1" else f"bomb.00{int(name[-1])-1}"

    def set_pose(self, pose: PoseStamped):
        self.pose_stamped = pose

    def distance_to(self, other):
        # Euclidean distance
        a = self.pose_stamped
        try:
            b = other.pose_stamped
        except AttributeError:
            b = other
        return ((a.pose.position.x - b.pose.position.x)**2 + (a.pose.position.y - b.pose.position.y)**2) ** 0.5

    def launch_bomb(self, target):
        try:
            import bge
        except ImportError:
            # Game engine hasn't started yet
            return

        robot = self.robots[target.data]
        scene = bge.logic.getCurrentScene()
        scene.objects[self.bomb_name].worldPosition = scene.objects[robot.name].worldPosition
        self.explode_target = robot.pose_stamped
        rospy.Timer(rospy.Duration(bomb_duration), self.explode)

        #self.bomb_pub.publish(robot.pose_stamped)

    def explode(self, _):
        try:
            import bge
        except ImportError:
            # Game engine hasn't started yet
            return

        scene = bge.logic.getCurrentScene()
        scene.objects[self.bomb_name].worldPosition.z = -10
        for robot in list(self.robots.values()):
            if robot.distance_to(self.explode_target) < bomb_range:
                scene.objects[robot.name].endObject()

    def add_lidar_sensor(self):
        self.lidar = Hokuyo()
        self.lidar.translate(x=0.275, z=0.252)
        self.append(self.lidar)
        self.lidar.properties(Visible_arc=False)
        self.lidar.properties(laser_range=30.0)
        self.lidar.properties(resolution=1)
        self.lidar.properties(scan_window=180.0)
        self.lidar.create_laser_arc()
        self.lidar.add_interface('ros', topic=f"{self.name}/lidar", frame_id=f"{self.name}/base_footprint")

    def add_motion_actuator(self):
        self.motion = MotionVWDiff()
        self.append(self.motion)
        self.motion.add_interface('ros', topic=f"{self.name}/cmd_vel")

    def add_pose_sensor(self):
        # Current position
        self.pose = Pose()
        self.append(self.pose)
        self.pose.add_interface('ros', topic=f"{self.name}/pose", frame_id=f"map")

    def add_odom_sensor(self):
        odometry = Odometry()
        self.append(odometry)
        odometry.add_interface('ros', topic=f"{self.name}/odom", frame_id=f"{self.name}/odom", child_frame_id=f"{self.name}/base_footprint")

    def add_to_simulation(self, position, rotation):
        self.x = position["x"]
        self.y = position["y"]
        self.z = position["z"]
        self.translate(position["x"], position["y"], position["z"])
        self.rotate(rotation["x"], rotation['y'], rotation['z'])
        self.add_pose_sensor()
        self.add_motion_actuator()
        self.add_odom_sensor()
        self.add_lidar_sensor()

        self.bomb = PassiveObject('/bomberman_ws/bomberman/map/bomb.blend', 'bomb')
        self.bomb.translate(x=self.x, y=self.y, z=10)
        
        self.robots[self.name] = self
