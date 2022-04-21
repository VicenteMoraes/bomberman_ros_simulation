import rospy
from geometry_msgs.msg import PoseStamped
from morse.builder import *

bomb_range = 2
bomb_duration = 5

class PioneerRobot(Pioneer3DX):
    number_of_robots = 3
    robots = []

    def __init__(self, name: str = ""):
        Pioneer3DX.__init__(self, name)
        self.name = name
        self.pose = PoseStamped()
        self.cmd_vel = 0
        self.pose_sub = rospy.Subscriber(f"/{self.name}/pose", PoseStamped, self.set_pose)
        self.bomb_pub = rospy.Publisher("bomb", PoseStamped, 1)

    def distance_to(self, other):
        # Euclidean distance
        a = self.pose
        try:
            b = other.pose
        except AttributeError:
            b = other
        return ((a.pose.position.x - b.pose.position.x)**2 + (a.pose.position.y - b.pose.position.y)**2) ** 0.5

    def get_closest_robot(self):
        closest = (None, float('inf'))
        for neighbour in self.robots:
            if self == neighbour:
                continue
            distance = self.distance_to(neighbour)
            if distance  < closest[1]:
                closest = (neighbour, distance)
        return closest

    def set_pose(self, pose: PoseStamped):
        self.pose = pose
        closest_robot, distance = self.get_closest_robot()
        if distance < bomb_range + 1:
            self.launch_bomb(closest_robot.pose)

    def launch_bomb(self, position: PoseStamped):
        try:
            import bge
        except ImportError:
            # Game engine hasn't started yet
            return

        scene = bge.logic.getCurrentScene()
        obj = scene.addObject("/bomberman_ws/bomberman/map/bomb.blender", time=0)
        rospy.Timer(rospy.Duration(bomb_duration), self.explode, (position, obj))

        self.bomb_pub.publish(position)

    def explode(self, bomb_position, blender_obj):
        try:
            import bge
        except ImportError:
            # Game engine hasn't started yet
            return

        scene = bge.logic.getCurrentScene()
        obj.endObject()
        for robot in self.robots:
            if robot.distance_to(bomb_position) < bomb_range
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
        
        self.robots.append(self)
