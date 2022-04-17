#! /usr/bin/env morseexec

from morse.builder import *
from bomberman.robot import PioneerRobot

if __name__ == "__main__":
    robot1 = PioneerRobot(name="robot1")
    robot2 = PioneerRobot(name="robot2")
    robot3 = PioneerRobot(name="robot3")

    robot1.add_to_simulation(position={'x': 7, 'y': -7, 'z': 0.1}, rotation={'x': 0, 'y': 0, 'z': 0})
    robot2.add_to_simulation(position={'x': 3, 'y': 0, 'z': 0.1}, rotation={'x': 0, 'y': 0, 'z': 0})
    robot3.add_to_simulation(position={'x': 3, 'y': 5, 'z': 0.1}, rotation={'x': 0, 'y': 0, 'z': 0})

    env = Environment('/bomberman_ws/bomberman/map/map.blend')
    env.set_camera_location([10, -10, 10])
    env.set_camera_rotation([1.0470, 0, 0.7854])
