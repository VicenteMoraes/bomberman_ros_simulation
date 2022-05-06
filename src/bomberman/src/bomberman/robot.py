import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from morse.builder import *
from bomberman.bomb import Bomb
import random

class PioneerRobot(Pioneer3DX):
    robots = {}

    def __init__(self, name: str = ""):
        Pioneer3DX.__init__(self, name)
        self.name = name
        self.pose_stamped = PoseStamped()
        self.pose_sub = rospy.Subscriber(f"/{self.name}/pose", PoseStamped, self.set_pose)
        self.kill_pub = rospy.Publisher(f"/{self.name}/kill", Bool, queue_size=1)
        self.bomb_target_sub = rospy.Subscriber(f"/{self.name}/bomb_target", String, self.launch_bomb)
        self.startup = False

    def set_pose(self, pose: PoseStamped):
        self.pose_stamped = pose
        if not self.startup:
            try:
                import bge
            except ImportError:
                return

            scene = bge.logic.getCurrentScene()
            obj = scene.objects[f"cube{self.name[-1]}"]
            robot = scene.objects[self.name]
            obj.worldPosition = [robot.worldPosition[0], robot.worldPosition[1], robot.worldPosition[2] + 0.1]
            obj.setParent(robot, False, True)
            self.startup = True

    def distance_to(self, other):
        # Euclidean distance
        a = self.pose_stamped
        try:
            b = other.pose_stamped
        except AttributeError:
            b = other
        return ((a.pose.position.x - b.pose.position.x)**2 + (a.pose.position.y - b.pose.position.y)**2) ** 0.5

    def launch_bomb(self, target):
        self.bomb.launch(target)

    def add_lidar_sensor(self):
        self.lidar = Hokuyo()
        #self.lidar.translate(x=0.275, z=0.252)
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

        self.bomb = Bomb(robot=self, name=f"bomb{self.name[-1]}")
        self.cube = PassiveObject('/bomberman_ws/bomberman/map/block.blend', f"cube{self.name[-1]}")
        self.cube.translate(x=int(self.name[-1]), y=0, z=-5)

        self.robots[self.name] = self
