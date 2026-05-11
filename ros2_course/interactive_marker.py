import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from scipy.spatial.transform import Rotation as R

class DVRKInteractiveMarker(Node):
    def __init__(self, position, shape='cylinder'):
        super().__init__('dvrk_interactive_marker_node')
        
        self.get_logger().info("Initializing dVRK Interactive Marker Server Architecture...")
        self.shape = shape
        self.target_frame = 'PSM1_psm_base_link'

        # Instantiate the interactive marker server bounded to the node's execution context
        self.server = InteractiveMarkerServer(self, 'dvrk_interactive_marker')

        # Publisher established to broadcast the live spatial pose to the kinematic controller
        self.pose_pub = self.create_publisher(PoseStamped, '/dvrk_viz/interactive_target_pose', 10)

        # Subscriber to allow the robot to drag the marker
        self.force_pose_sub = self.create_subscription(PoseStamped, '/dvrk_viz/force_marker_pose', self.cb_force_pose, 10)

        self.current_marker_pose = None
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Execute marker construction protocols
        self.create_interactive_marker(position)

    def create_interactive_marker(self, initial_position):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.target_frame
        int_marker.header.stamp = self.get_clock().now().to_msg()
        int_marker.name = "dvrk_grasp_target"
        int_marker.description = "6-DOF Graspable & Traceable Target"
        int_marker.scale = 0.08  # Dimensional scale of the surrounding 6-DOF control axes

        # Define the initial Cartesian translation coordinates
        int_marker.pose.position.x = float(initial_position[0])
        int_marker.pose.position.y = float(initial_position[1])
        int_marker.pose.position.z = float(initial_position[2])
        
        # Define initial orientation utilizing Scipy spatial transforms to avert gimbal lock
        r = R.from_euler('x', -20, degrees=True)
        quat = r.as_quat()
        int_marker.pose.orientation = Quaternion(
            x=float(quat[0]), 
            y=float(quat[1]), 
            z=float(quat[2]), 
            w=float(quat[3])
        )
        
        # Construct the core visual representation mesh
        visual_marker = Marker()
        if self.shape == 'sphere':
            visual_marker.type = Marker.SPHERE
            visual_marker.scale.x = 0.015
            visual_marker.scale.y = 0.015
            visual_marker.scale.z = 0.015
        else: # Cylindrical disk geometry
            visual_marker.type = Marker.CYLINDER
            visual_marker.scale.x = 0.12   # 12 cm diametric span
            visual_marker.scale.y = 0.12
            visual_marker.scale.z = 0.002  # 2 mm vertical thickness

        # Solid Green with full opacity
        visual_marker.color.r = 0.0
        visual_marker.color.g = 1.0
        visual_marker.color.b = 0.0
        visual_marker.color.a = 1.0

        # Encapsulate the visual marker within a non-interactive control
        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True
        visual_control.markers.append(visual_marker)
        visual_control.interaction_mode = InteractiveMarkerControl.NONE
        int_marker.controls.append(visual_control)

        # Inject the 6-DOF translation and rotation controls into the marker structure
        self.add_6dof_controls(int_marker)

        # Register the marker within the server registry and bind the feedback execution callback
        self.server.insert(int_marker, feedback_callback=self.process_feedback)
        
        # Serialize state modifications and broadcast the update over the DDS network
        self.server.applyChanges()
        self.get_logger().info("Interactive Marker serialized and applied to visualization client.")

        self.current_marker_pose = int_marker.pose

    def add_6dof_controls(self, int_marker):
        """Programmatically appends spatial X, Y, and Z translation and rotation rings."""
        
        # X-Axis Spatial Controls
        control_x = InteractiveMarkerControl()
        control_x.name = "move_x"
        control_x.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=1.0) 
        control_x.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control_x)
        
        control_x_rot = InteractiveMarkerControl()
        control_x_rot.name = "rotate_x"
        control_x_rot.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=1.0)
        control_x_rot.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control_x_rot)

        # Y-Axis Spatial Controls (Requires 90 degree coordinate transformation)
        control_y = InteractiveMarkerControl()
        control_y.name = "move_y"
        control_y.orientation = Quaternion(x=0.0, y=0.0, z=1.0, w=1.0)
        control_y.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control_y)

        control_y_rot = InteractiveMarkerControl()
        control_y_rot.name = "rotate_y"
        control_y_rot.orientation = Quaternion(x=0.0, y=0.0, z=1.0, w=1.0)
        control_y_rot.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control_y_rot)

        # Z-Axis Spatial Controls (Requires 90 degree coordinate transformation)
        control_z = InteractiveMarkerControl()
        control_z.name = "move_z"
        control_z.orientation = Quaternion(x=0.0, y=1.0, z=0.0, w=1.0)
        control_z.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control_z)

        control_z_rot = InteractiveMarkerControl()
        control_z_rot.name = "rotate_z"
        control_z_rot.orientation = Quaternion(x=0.0, y=1.0, z=0.0, w=1.0)
        control_z_rot.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control_z_rot)

    def process_feedback(self, feedback):
        """Callback subroutine triggered by the DDS network upon user interaction events."""
        p = feedback.pose.position
        
        # Log the interaction event to standard output for debugging spatial coordinate flow
        self.get_logger().debug(f"Marker Translated/Rotated -> Pos: [{p.x:.3f}, {p.y:.3f}, {p.z:.3f}]")

        # Saving Pose for continuous publishing in the timer callback
        self.current_marker_pose = feedback.pose

    def timer_callback(self):
        """Continous publishing of the current pose, ensuring that late-arriving nodes wake up promptly."""
        if self.current_marker_pose is not None:
            self.publish_current_pose(self.current_marker_pose)

    def publish_current_pose(self, pose):
        """Constructs and publishes a validated PoseStamped message to the hardware/simulation topics."""
        msg = PoseStamped()
        msg.header.frame_id = self.target_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = pose
        self.pose_pub.publish(msg)
    
    def cb_force_pose(self, msg):
        """Programmatically updates the marker position in RViz."""
        self.server.setPose("dvrk_grasp_target", msg.pose)
        self.server.applyChanges()
        self.current_marker_pose = msg.pose

def main(args=None):
    rclpy.init(args=args)
    target_center = [0.0, 0.12, -0.12]
    
    # Initialize the architecture. Shape defaults to cylinder for the circular edge tracing task.
    interactive_node = DVRKInteractiveMarker(target_center, shape='cylinder')
    
    try:
        rclpy.spin(interactive_node)
    except KeyboardInterrupt:
        interactive_node.get_logger().info('Interrupt received. Terminating Interactive Marker Server...')
    finally:
        interactive_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()