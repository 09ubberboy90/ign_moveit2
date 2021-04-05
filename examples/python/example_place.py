#!/usr/bin/env python3

import threading
from geometry_msgs.msg import Pose
from moveit2 import MoveIt2Interface
from rclpy.node import Node
import rclpy
import time
import copy


class Thrower(Node):

    def __init__(self):
        super().__init__("thrower")
        # Create a subscriber for object pose
        self._object_pose_sub = self.create_subscription(Pose, '/model/throwing_object/pose',
                                                         self.object_pose_callback, 1)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.pose = None
        # Create MoveIt2 interface node
        self._moveit2 = MoveIt2Interface()
        self.bool = False
        # Create multi-threaded executor
        self._executor = rclpy.executors.MultiThreadedExecutor(2)
        self._executor.add_node(self)
        self._executor.add_node(self._moveit2)
        # Wait a couple of seconds until Ignition is ready and spin up the executor
        time.sleep(2)
        self._executor.spin()

    def timer_callback(self):
        if self.bool:
            self.timer.destroy()
            self.throw(copy.deepcopy(self.pose))

    def object_pose_callback(self, pose_msg):
        self.pose = pose_msg.position
        self.bool = True

    def throw(self, object_position):
        # Open gripper
        self._moveit2.gripper_open()
        self._moveit2.wait_until_executed()

        initial = self._moveit2.get_joint_state()
        # Move above object
        position = [object_position.x,
                    object_position.y, object_position.z + 0.1]
        quaternion = [1.0, 0.0, 0.0, 0.0]
        self._moveit2.set_pose_goal(position, quaternion)
        self._moveit2.plan_kinematic_path()
        self._moveit2.execute()
        self._moveit2.wait_until_executed()

        self._moveit2.set_max_velocity(0.5)
        self._moveit2.set_max_acceleration(0.5)

        # Move to grasp position
        position = [object_position.x,
                    object_position.y, object_position.z]
        quaternion = [1.0, 0.0, 0.0, 0.0]
        self._moveit2.set_pose_goal(position, quaternion)
        self._moveit2.plan_kinematic_path()
        self._moveit2.execute()
        self._moveit2.wait_until_executed()

        # Close gripper
        self._moveit2.gripper_close(width=0.05, speed=0.2, force=20.0)
        self._moveit2.wait_until_executed()

        # Move above object again
        position = [object_position.x,
                    object_position.y, object_position.z + 0.1]
        quaternion = [1.0, 0.0, 0.0, 0.0]
        self._moveit2.set_pose_goal(position, quaternion)
        self._moveit2.plan_kinematic_path()
        self._moveit2.execute()
        self._moveit2.wait_until_executed()

        # move to new position
        position = [object_position.x+0.2,
                    object_position.y, object_position.z]
        quaternion = [1.0, 0.0, 0.0, 0.0]
        self._moveit2.set_pose_goal(position, quaternion)
        self._moveit2.plan_kinematic_path()
        self._moveit2.execute()
        self._moveit2.wait_until_executed()

        self._moveit2.gripper_open()
        self._moveit2.wait_until_executed()

        joint_positions = [0.0,
                           0.0,
                           0.0,
                           0.0,
                           0.0,
                           1.571,
                           0.785, ]
        self._moveit2.set_joint_goal(joint_positions)
        self._moveit2.plan_kinematic_path()
        self._moveit2.execute()
        self._moveit2.wait_until_executed()
        new_pose = self.pose
        if ((new_pose.x < object_position.x - 0.05) or (object_position.x + 0.05 < new_pose.x)):
            self.get_logger().info("Cube is not in bound")
        else:
            self.get_logger().info("Task completed Succesfully")


def main(args=None):
    rclpy.init(args=args)

    _thrower = Thrower()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
