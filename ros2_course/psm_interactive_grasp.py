import rclpy
import time
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

class PSMDynamicController(Node):
    def __init__(self):
        super().__init__('psm_interactive_grasp')

        self.measured_cp = None
        self.measured_jaw = None
        
        # Dynamic marker state variables updated via DDS callback
        self.target_marker_pose = None
        self.target_marker_radius = 0.06 # Default initialized for a 12cm cylinder

        self.is_synchronized = False

        # Hardware/Simulator telemetry subscriptions
        self.measured_cp_sub = self.create_subscription(PoseStamped, '/PSM1/measured_cp', self.cb_measured_cp, 10)
        self.jaw_sub = self.create_subscription(JointState, '/PSM1/jaw/measured_js', self.cb_jaw_measured_js, 10)
        self.marker_sub = self.create_subscription(PoseStamped, '/dvrk_viz/interactive_target_pose', self.cb_interactive_marker, 10) # Interactive Marker telemetry

        # Hardware/Simulator command publishers
        self.servo_cp_pub = self.create_publisher(PoseStamped, '/PSM1/servo_cp', 10)
        self.servo_jaw_pub = self.create_publisher(JointState, '/PSM1/jaw/servo_jp', 10)
        self.force_marker_pub = self.create_publisher(PoseStamped, '/dvrk_viz/force_marker_pose', 10)

    def cb_measured_cp(self, msg):
        self.measured_cp = msg

    def cb_jaw_measured_js(self, msg):
        self.measured_jaw = msg

    def cb_interactive_marker(self, msg):
        """Asynchronously updates the internal state with the latest spatial marker pose."""
        self.target_marker_pose = msg.pose

    def wait_for_data_synchronization(self):
        """Validates that initial telemetry streams are established prior to task execution."""

        if self.is_synchronized:
            return

        self.get_logger().info('Synchronizing PSM telemetry and interactive marker spatial data...')
        while rclpy.ok() and (self.measured_cp is None or self.measured_jaw is None or self.target_marker_pose is None):
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('Data synchronization successfully established.')
        self.is_synchronized = True

    def set_jaw_angle(self, target, omega, dt):
        """Activates the PSM jaw mechanism using linear temporal interpolation with a safety timeout."""

        self.measured_jaw = None #fresh state
        
        # Timeout so the node doesn't hang indefinitely if the topic dies
        timeout_start = time.time()
        while rclpy.ok() and self.measured_jaw is None:
            rclpy.spin_once(self, timeout_sec=0.01)
            if time.time() - timeout_start > 2.0:
                self.get_logger().error("Timeout: Failed to receive jaw telemetry.")
                return

        start_angle = self.measured_jaw.position[0]
        distance = abs(target - start_angle)
        
        if distance < 0.0001:
            return

        t_total = distance / omega # time = distance / speed (velocity)      
        num = max(int(t_total / dt), 2) # discrete steps, at least 2 to ensure movement
        jaw_angles = np.linspace(start_angle, target, num) # array of angles to publish for smooth interpolation

        for angle in jaw_angles:
            if not rclpy.ok(): break
            
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.position = [float(angle)]
            self.servo_jaw_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            self.smart_sleep(dt)

    def get_dynamic_target(self, target_type, z_offset, theta=0.0, absolute_goal=None):
        """Calculates the current global target pose based on the latest marker data or absolute goal."""

        if target_type == 'absolute' and absolute_goal is not None:
            p_target = np.array(absolute_goal)
            r_flip = R.from_euler('x', 180, degrees=True) # The orientation should face downwards (or as requested)
            return p_target, r_flip
        
        # Extract marker's global center (vector) and orientation (rotation matrix)
        target_pose = self.target_marker_pose
        center = np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z])
        r_marker = R.from_quat([target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w])
        
        if target_type == 'center':
            local_point = np.array([0.0, 0.0, z_offset])
        elif target_type == 'circle':
            local_point = np.array([
                self.target_marker_radius * np.cos(theta),
                self.target_marker_radius * np.sin(theta),
                z_offset
            ])
        else:
            self.get_logger().error(f"Unknown target_type: {target_type}")
            return None, None

        # Apply marker rotation to local offset to get global target point
        p_target = center + r_marker.apply(local_point)
        
        # Orient the end-effector downward
        r_flip = R.from_euler('x', 180, degrees=True) 
        r_target = r_marker * r_flip 
        
        return p_target, r_target

    def dynamic_move_to_target(self, v, dt, target_type='center', z_offset=0.0, theta=0.0, tolerance=0.002, absolute_goal=None, drag_marker=False):
        """
        Centralized Visual Servoing Implementation.
        Continuously calculates and chases a dynamic target ('center' or 'circle').
        """
        self.wait_for_data_synchronization()
        last_ui_update = time.monotonic() # Monotonic time for real-world elapsed time tracking --> unaffected by ROS2 clock changes
        
        while rclpy.ok():
            # 1. CONTINUOUSLY read the physical state to prevent mathematical drift
            current_pose = self.measured_cp.pose
            p_real = np.array([current_pose.position.x, current_pose.position.y, current_pose.position.z])
            r_real = R.from_quat([current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w])

            # 2. Get dynamic target
            p_target, r_target = self.get_dynamic_target(target_type, z_offset, theta, absolute_goal)
            
            # 3. Compare TARGET to REAL position (not theoretical position)
            vector_diff = p_target - p_real
            distance = np.linalg.norm(vector_diff)

            if distance < tolerance:
                break

            # 4. Mathematically update our command relative to our REAL position
            step_size = min(v * dt, distance)
            direction = vector_diff / distance
            p_cmd = p_real + (direction * step_size)

            # 5. Mathematically update orientation relative to REAL orientation
            # Spherical Linear Interpolation (SLERP) for smooth 3D rotation
            slerp_fraction = step_size / distance 
            key_times = [0.0, 1.0]
            key_rots = R.from_quat([r_real.as_quat(), r_target.as_quat()])
            slerp = Slerp(key_times, key_rots)
            interp_quat = slerp([slerp_fraction]).as_quat()[0]
            
            self.publish_cmd(p_cmd, interp_quat)

            if drag_marker:
                current_time = time.monotonic()
                if (current_time - last_ui_update) >= 0.05: # Update marker at most every 50ms to reduce network load
                    last_ui_update = current_time # Reset the timer
                    marker_msg = PoseStamped()
                    marker_msg.header.frame_id = 'PSM1_base'
                    marker_msg.header.stamp = self.get_clock().now().to_msg()
                    
                    # Apply the current robot position (plus a tiny z-offset so it sits nicely inside the jaws)
                    marker_msg.pose.position.x = float(p_real[0])
                    marker_msg.pose.position.y = float(p_real[1])
                    marker_msg.pose.position.z = float(p_real[2] - 0.01) # 1cm offset fromthe jaws
                    
                    # Keep the marker's original flat orientation so it doesn't spin wildly
                    marker_msg.pose.orientation = self.target_marker_pose.orientation
                    
                    self.force_marker_pub.publish(marker_msg)

            self.smart_sleep(dt)

    def publish_cmd(self, pos, quat):
        """Helper function for constructing and publishing PoseStamped messages."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'PSM1_base'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, pos)
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = map(float, quat)
        
        self.servo_cp_pub.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.0)

    def smart_sleep(self, duration_sec):
        """Sleeps for a duration while keeping the ROS2 network alive."""
        # Use monotonic (real-world) time to avoid simulation clock freezes
        start_time = time.monotonic()
        while rclpy.ok() and (time.monotonic() - start_time) < duration_sec:
            rclpy.spin_once(self, timeout_sec=0.001) # check once and move on --> lower CPU usage 
            time.sleep(0.001) # Give the CPU a tiny breath so WSL doesn't disconnect
            
    def surgical_cutting_task(self, velocity, dt, steps=16, z_offset=0.006):
        """Sectional cutting task: the robot performs a circumferential cut by moving along a circular path, stopping at each point to execute a "cut" action."""
        self.wait_for_data_synchronization()
        self.get_logger().info(f'Initiating surgical tissue excision: {steps} cutting points...')

        theta = 0.0
        angle_step = (2 * np.pi) / steps
        
        for i in range(steps + 1):
            if not rclpy.ok(): break

            self.dynamic_move_to_target(v=velocity, dt=dt, target_type='circle', z_offset=z_offset, theta=theta, tolerance=0.001)

            self.set_jaw_angle(target=0.0, omega=2.0, dt=0.01) # Close (Cut)
            self.set_jaw_angle(target=0.6, omega=2.0, dt=0.01) # Open

            theta += angle_step

        self.get_logger().info('Circumferential cutting completed.')

    def execute_surgical_cutting_demo(self):
        """Control of the complete surgical intervention: cutting, grasping, removal."""
        self.get_logger().info('--- Starting Surgical Tissue Excision Demo ---')
        
        # 1. Initial state: Open forceps
        self.set_jaw_angle(0.6, omega=1.0, dt=0.01)
        
        # 2. Moving to the incision start point
        self.get_logger().info('Moving to incision start point...')
        self.dynamic_move_to_target(v=0.05, dt=0.01, z_offset=0.02)
        
        # 3. Circumferential cutting
        self.surgical_cutting_task(velocity=0.04, dt=0.01, steps=24)

        # 4. Moving to the geometric center for extraction
        self.get_logger().info('Moving to geometric center for extraction...')
        self.dynamic_move_to_target(v=0.04, dt=0.01, target_type='center', z_offset=0.03) # First moves above
        self.dynamic_move_to_target(v=0.02, dt=0.01, target_type='center', z_offset=0.005) # Then lowers down
        
        # 5. Grasping and extracting tissue
        self.get_logger().info('Grasping and extracting tissue...')
        self.set_jaw_angle(0.0, omega=0.5, dt=0.01) # Secure grasp
        
        # Extraction to the absolute point
        self.dynamic_move_to_target(v=0.03, dt=0.01, target_type='absolute', absolute_goal=[0.0, 0.0, -0.07], drag_marker=True)

        self.get_logger().info('--- Procedure Successfully Completed ---')
        self.smart_sleep(0.5) # Allow final commands to be sent before shutdown
    
    def boundary_test(self):
        self.dynamic_move_to_target(v=0.03, dt=0.01, target_type='absolute', absolute_goal=[0.0, 0.1, -0.05])

def main(args=None):
    rclpy.init(args=args)
    node = PSMDynamicController()

    try:
        node.execute_surgical_cutting_demo()
        #node.boundary_test()
    except Exception as e:
        node.get_logger().error(f'Runtime fault encountered during execution: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# 3 külön terminálba:
    # ./ros2_ws/src/ros2_course/start_dvrk.sh

    # ros2 run ros2_course interactive_marker

    # ros2 run ros2_course psm_interactive_grasp