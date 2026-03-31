import rclpy
from geometry_msgs.msg import Pose

rclpy.init()
node = rclpy.create_node("camera")

pub = node.create_publisher(Pose,"/camera/target_pose",10)

msg = Pose()
msg.position.x = 0.25
msg.position.y = 0.0
msg.position.z = 0.05
msg.orientation.w = 1.0

pub.publish(msg)