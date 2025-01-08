# import rclpy
# from rclpy.node import Node
# from tf2_ros import TransformBroadcaster
# from geometry_msgs.msg import TransformStamped
# import tf2_ros
# from builtin_interfaces.msg import Time

# class FramePublisher(Node):
#     def __init__(self):
#         super().__init__('frame_publisher')
#         self.tf_broadcaster = TransformBroadcaster(self)

#         # Create and publish transform from base_link to camera_link_optical
#         self.create_timer(0.1, self.broadcast_transform)

#     def broadcast_transform(self):
#         # Create TransformStamped object for the base_link to camera_link_optical transform
#         transform = TransformStamped()

#         # Header information
#         transform.header.stamp = self.get_clock().now().to_msg()
#         transform.header.frame_id = 'base_link'  # Parent frame
#         transform.child_frame_id = 'camera_link_optical'  # Child frame

#         # Define the translation (position of the camera with respect to the base_link)
#         transform.transform.translation.x = 0.5  # Example value (meters)
#         transform.transform.translation.y = 0.0  # Example value (meters)
#         transform.transform.translation.z = 0.0  # Example value (meters)

#         # Define the rotation (orientation of the camera with respect to the base_link)
#         transform.transform.rotation.x = 0.0#0.7071#0.0
#         transform.transform.rotation.y = 0.0
#         transform.transform.rotation.z = 0.0
#         transform.transform.rotation.w = 1.0 #0.7071#1.0

#         # Broadcast the transform
#         self.tf_broadcaster.sendTransform(transform)

# def main(args=None):
#     rclpy.init(args=args)

#     frame_publisher = FramePublisher()

#     rclpy.spin(frame_publisher)

#     frame_publisher.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()






import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class CameraTransformBroadcaster(Node):
    def __init__(self):
        super().__init__('camera_transform_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)

        # Create a timer to continuously broadcast the transform
        self.create_timer(0.001, self.broadcast_transform)
        self.get_logger().info("tf broadcaster have been started")
        
    def broadcast_transform(self):
        

        # Create a TransformStamped message for the transform
        transform2 = TransformStamped()

        # Fill in the header information
        transform2.header.stamp = self.get_clock().now().to_msg()
        transform2.header.frame_id = 'base_link'  # Parent frame
        transform2.child_frame_id = 'camera_link_temp'  # Child frame

        # Define the translation (position) of camera_link_optical relative to camera_depth_optical_frame
        transform2.transform.translation.x = 1.0  # Example translation in meters
        transform2.transform.translation.y = 0.0  # Example translation in meters
        transform2.transform.translation.z = 0.0  # Example translation in meters

        # Define the rotation (orientation) of camera_link_optical relative to camera_depth_optical_frame
        transform2.transform.rotation.x = 0.0
        transform2.transform.rotation.y = 0.7071
        transform2.transform.rotation.z = 0.0
        transform2.transform.rotation.w = 0.7071 # Identity rotation (no rotation)

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform2)

        # Create a TransformStamped message for the transform
        transform3 = TransformStamped()

        # Fill in the header information
        transform3.header.stamp = self.get_clock().now().to_msg()
        transform3.header.frame_id = 'camera_link_temp'  # Parent frame
        transform3.child_frame_id = 'camera_link'  # Child frame

        # Define the translation (position) of camera_link_optical relative to camera_depth_optical_frame
        transform3.transform.translation.x = 0.0  # Example translation in meters
        transform3.transform.translation.y = 0.0  # Example translation in meters
        transform3.transform.translation.z = 0.0  # Example translation in meters

        # Define the rotation (orientation) of camera_link_optical relative to camera_depth_optical_frame
        transform3.transform.rotation.x = 0.0
        transform3.transform.rotation.y = 0.0
        transform3.transform.rotation.z = -0.7071
        transform3.transform.rotation.w = 0.7071 # Identity rotation (no rotation)

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform3)

        # ====================================================================
        # Create a TransformStamped message for the transform
        transform2 = TransformStamped()
                # Fill in the header information
        
        transform2.header.stamp = self.get_clock().now().to_msg()
        transform2.header.frame_id = 'camera_link'  # Parent frame
        transform2.child_frame_id = 'camera_depth_optical_frame'  # Child frame

        # Define the translation (position) of camera_link_optical relative to camera_depth_optical_frame
        transform2.transform.translation.x = 0.0  # Example translation in meters
        transform2.transform.translation.y = 0.0  # Example translation in meters
        transform2.transform.translation.z = 0.0  # Example translation in meters

        # Define the rotation (orientation) of camera_link_optical relative to camera_depth_optical_frame
        transform2.transform.rotation.x = 0.0
        transform2.transform.rotation.y = 0.0
        transform2.transform.rotation.z = 0.0
        transform2.transform.rotation.w = 1.0  # Identity rotation (no rotation)

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform2)


# Create a TransformStamped message for the transform
        transform = TransformStamped()

        # Fill in the header information
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'camera_depth_optical_frame'  # Parent frame
        transform.child_frame_id = 'camera_link_optical'  # Child frame

        # Define the translation (position) of camera_link_optical relative to camera_depth_optical_frame
        transform.transform.translation.x = 0.0  # Example translation in meters
        transform.transform.translation.y = 0.0  # Example translation in meters
        transform.transform.translation.z = 0.0  # Example translation in meters

        # Define the rotation (orientation) of camera_link_optical relative to camera_depth_optical_frame
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0  # Identity rotation (no rotation)

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(transform)


        self.get_logger().info("ok123")
        

def main(args=None):
    rclpy.init(args=args)

    # Create and spin the transform broadcaster node
    node = CameraTransformBroadcaster()
    rclpy.spin(node)

    # Clean up after node is done
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
