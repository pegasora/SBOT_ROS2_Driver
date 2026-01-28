from launch import LaunchDescription
from launch_ros.actions import Node
import sys

name = "dave"

for arg in sys.argv:
    if arg.startswith("robot_name:="):
        name = arg.split(":=")[1]


def generate_launch_description():
    ld = LaunchDescription()

    #  you will need to change this to match your robot if not in CDA
    ip = "http://10.8.4.11:3000"
    token = "8geqfqu0-qbbkig-ozwgr4-tl2xfj7"

    cart_pose_action_server = Node(
        package="sb_controller",
        executable="standard_bot_set_cart_pose_server_node",
        parameters=[{"robot_url": ip}, {"robot_token": token}, {"robot_name": name}],
    )

    joint_rot_action_server = Node(
        package="sb_controller",
        executable="standard_bot_set_joint_rot_server_node",
        parameters=[{"robot_url": ip}, {"robot_token": token}, {"robot_name": name}],
    )

    on_robot_gripper_action_server = Node(
        package="sb_controller",
        executable="standart_bot_on_robot_gripper_server_node",
        parameters=[{"robot_url": ip}, {"robot_token": token}, {"robot_name": name}],
    )

    ld.add_action(cart_pose_action_server)
    ld.add_action(joint_rot_action_server)
    ld.add_action(on_robot_gripper_action_server)

    return ld
