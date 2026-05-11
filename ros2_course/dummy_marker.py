import rclpy
import numpy as np
from rclpy.node import Node
from visualization_msgs.msg import Marker
from scipy.spatial.transform import Rotation as R

class DummyMarker(Node):
    def __init__(self, position, shape='cylinder'):
        super().__init__('dummy_marker_publisher')
        self.position = position
        self.shape = shape

        # Létrehozunk egy publikálót a 'dummy_target_marker' topicra
        self.publisher_ = self.create_publisher(Marker, 'dummy_target_marker', 10)
        timer_period = 0.1  # másodperc (10Hz-en frissít)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        marker = Marker()
        # Referencia keret - ehhez képest jelenik meg a gömb
        marker.header.frame_id = 'PSM1_psm_base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "dvrk_viz"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.MODIFY
        # self.i = 0
        
        # Pozíció beállítása a kapott lista alapján
        marker.pose.position.x = float(self.position[0])
        marker.pose.position.y = float(self.position[1])
        marker.pose.position.z = float(self.position[2])
        r = R.from_euler('x', -40, degrees=True)
        quat = r.as_quat() # [x, y, z, w]

        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        
        # Forma és méret shape alapján
        if self.shape == 'sphere':
            # Gömb beállításai
            marker.type = Marker.SPHERE
            marker.scale.x = 0.008
            marker.scale.y = 0.008
            marker.scale.z = 0.008
        elif self.shape == 'cylinder':
            # Lapos korong beállításai
            marker.type = Marker.CYLINDER
            marker.scale.x = 0.12   # 6 cm átmérős
            marker.scale.y = 0.12
            marker.scale.z = 0.002  # 2 mm vastagság
        else:
            self.get_logger().warn('Ismeretlen forma! Henger lesz az alapértelmezett.')
            marker.type = Marker.CYLINDER
            marker.scale.x = 0.06
            marker.scale.y = 0.06
            marker.scale.z = 0.002
        
        # Szín: RGBA (Alpha = átlátszóság, G = Zöld) -> Ez egy teljesen zöld gömb lesz
        marker.color.a = 1.0 
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.publisher_.publish(marker)
        # self.i += 1

def main(args=None):
    rclpy.init(args=args)
    
    # A körpálya középpontja
    target_center = [0, 0.1, -0.12]
    # shape='cylinder' -> Lapos korong 
    # shape='sphere'   -> Kis gömb (grasp)
    marker_publisher = DummyMarker(target_center, shape='cylinder')
    
    try:
        rclpy.spin(marker_publisher)
    except KeyboardInterrupt:
        # Ctrl+C megnyomásakor küldünk egy törlő üzenetet
        marker_publisher.get_logger().info('Markerek törlése...')
        clear_msg = Marker()
        clear_msg.header.frame_id = 'PSM1_psm_base_link'
        clear_msg.ns = "dvrk_viz"
        clear_msg.action = Marker.DELETEALL # Ez törli az összeset az adott névtérben
        marker_publisher.publisher_.publish(clear_msg)
    
    marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()