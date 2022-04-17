import rospy
from geometry_msgs.msg import PoseStamped
from morse.builder import *


class PioneerRobot(Pioneer3DX):
    number_of_robots = 3

    def __init__(self, name: str = ""):
        Pioneer3DX.__init__(self, name)
        self.name = name
        self.pose = PoseStamped()
        self.cmd_vel = 0
        self.my_pose = rospy.Subscriber(f"/{self.name}/pose", PoseStamped, self.set_pose)
        self.bomb_pub = rospy.Publisher("bomb", PoseStamped, 1)
        self.pose_subs = dict()

    def set_pose(self, pose: PoseStamped):
        self.pose = pose

    def launch_bomb(self, position: PoseStamped):
        self.bomb_pub.publish(position)

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

    def pose_callback(self, pose: PoseStamped):
        pass

    def add_pose_subscriber(self, robot_name: str):
        # Subscribes to another robot's pose topic
        self.pose_subs[robot_name] = rospy.Subscriber(f"{robot_name}/pose", PoseStamped, self.pose_callback)

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

        for i in range(1, self.number_of_robots+1):
            self.add_pose_subscriber(f"robot{i}")
