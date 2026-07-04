import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String             
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
import math
import time
import json

class AmrSimulator(Node):
    def __init__(self):
        super().__init__('amr_2sim')
        
        self.declare_parameter('map_file', '')
        map_file_path = self.get_parameter('map_file').value

        self.pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.walls = []
        
        if map_file_path:
            try:
                with open(map_file_path, 'r') as f:
                    world_data = json.load(f)
                    for w in world_data.get('walls', []):
                        p1 = (float(w[0][0]), float(w[0][1]))
                        p2 = (float(w[1][0]), float(w[1][1]))
                        self.walls.append((p1, p2))
                self.get_logger().info(f"Loaded world map from: {map_file_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to load map: {e}")
        
        if not self.walls:
            self.walls = [
                ((5.0, -5.0), (5.0, 5.0)),
                ((5.0, 5.0), (-5.0, 5.0)),
                ((-5.0, 5.0), (-5.0, -5.0)),
                ((-5.0, -5.0), (5.0, -5.0))
            ]

        self.cmd_vel = {'v': 0.0, 'w': 0.0}
        self.last_time = time.time()
        
        # พารามิเตอร์จำลองล้อหุ่นยนต์ (Robot Kinematics)
        self.wheel_base = 0.5         
        self.ticks_per_meter = 1000.0 

        # ----------------------------------------------------
        # 1. เพิ่มตัวแปรสำหรับเก็บค่า Encoder สะสม (Cumulative Pulses)
        # ----------------------------------------------------
        self.total_pulse_left = 0.0
        self.total_pulse_right = 0.0

        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.encoder_pub = self.create_publisher(String, '/wheel/encoder', 10)
        self.wheel_vel_pub = self.create_publisher(Twist, '/wheel/vel', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        self.timer = self.create_timer(0.05, self.timer_callback)
        
    def cmd_callback(self, msg):
        self.cmd_vel['v'] = msg.linear.x
        self.cmd_vel['w'] = msg.angular.z

    def timer_callback(self):
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0:
            dt = 0.05
            
        self.last_time = current_time

        new_theta = self.pose['theta'] + (self.cmd_vel['w'] * dt)
        new_x = self.pose['x'] + (self.cmd_vel['v'] * dt * math.cos(new_theta))
        new_y = self.pose['y'] + (self.cmd_vel['v'] * dt * math.sin(new_theta))

        if not self.check_collision(new_x, new_y):
            self.pose['x'] = new_x
            self.pose['y'] = new_y
        else:
            self.get_logger().warn("Collision Detected! Stopped translation.", throttle_duration_sec=1.0)
            
        self.pose['theta'] = new_theta

        # ----------------------------------------------------
        # จำลองการทำงานของ Encoder แบบสะสม (Cumulative)
        # ----------------------------------------------------
        v = self.cmd_vel['v']
        w = self.cmd_vel['w']
        
        v_right = v + (w * self.wheel_base / 2.0)
        v_left = v - (w * self.wheel_base / 2.0)
        
        # คำนวณระยะ pulse ย่อยในรอบนี้ (เก็บเป็น float เพื่อป้องกันค่าผิดเพี้ยนจากการปัดเศษทิ้ง)
        delta_pulse_right = v_right * dt * self.ticks_per_meter
        delta_pulse_left = v_left * dt * self.ticks_per_meter
        
        # 2. บวกสะสมเข้าไปในตัวแปรรวม
        self.total_pulse_right += delta_pulse_right
        self.total_pulse_left += delta_pulse_left
        
        sim_v_right = delta_pulse_right / (self.ticks_per_meter * dt)
        sim_v_left = delta_pulse_left / (self.ticks_per_meter * dt)
        
        wheel_v = (sim_v_right + sim_v_left) / 2.0
        wheel_w = (sim_v_right - sim_v_left) / self.wheel_base

        self.stamp = self.get_clock().now().to_msg()

        self.publish_tf()
        self.publish_scan()
        self.publish_odom()
        
        # 3. ส่งข้อมูล Topic ด้วยค่าสะสมที่แปลงเป็นจำนวนเต็ม (Integer)
        self.publish_encoder(int(self.total_pulse_left), int(self.total_pulse_right))
        self.publish_wheel_vel(wheel_v, wheel_w)
        self.publish_imu()

    def get_ray_intersection(self, x, y, angle, range_max):
        x2 = x + range_max * math.cos(angle)
        y2 = y + range_max * math.sin(angle)
        min_dist = float('inf')
        for wall in self.walls:
            x3, y3 = wall[0]
            x4, y4 = wall[1]
            den = (x - x2) * (y3 - y4) - (y - y2) * (x3 - x4)
            if den == 0:
                continue 
            t = ((x - x3) * (y3 - y4) - (y - y3) * (x3 - x4)) / den
            u = -((x - x2) * (y - y3) - (y - y2) * (x - x3)) / den
            if 0 <= t <= 1 and 0 <= u <= 1:
                dist = t * range_max
                if dist < min_dist:
                    min_dist = dist
        return min_dist

    def publish_encoder(self, left_pulse, right_pulse):
        msg = String()
        msg.data = f"ENC:{left_pulse},{right_pulse}"
        self.encoder_pub.publish(msg)

    def publish_wheel_vel(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.wheel_vel_pub.publish(msg)

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.stamp
        msg.header.frame_id = 'base_link'
        
        msg.orientation.z = math.sin(self.pose['theta'] / 2.0)
        msg.orientation.w = math.cos(self.pose['theta'] / 2.0)
        
        msg.angular_velocity.z = self.cmd_vel['w']
        
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81
        
        msg.orientation_covariance[0] = -1.0 
        msg.angular_velocity_covariance[0] = 0.01
        msg.linear_acceleration_covariance[0] = 0.01
        
        self.imu_pub.publish(msg)

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.pose['x']
        t.transform.translation.y = self.pose['y']
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.pose['theta'] / 2.0)
        t.transform.rotation.w = math.cos(self.pose['theta'] / 2.0)
        self.tf_broadcaster.sendTransform(t)

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.stamp
        scan.header.frame_id = 'laser_link'
        scan.angle_min = 0.0
        scan.angle_max = 2 * math.pi
        scan.angle_increment = math.radians(1.0) 
        scan.range_max = 12.0
        
        ranges = []
        intensities = []
        for i in range(360):
            laser_angle = self.pose['theta'] + (i * scan.angle_increment)
            dist = self.get_ray_intersection(self.pose['x'], self.pose['y'], laser_angle, scan.range_max)
            ranges.append(dist) 
            intensities.append(1.0)
        
        scan.ranges = ranges
        scan.intensities = intensities
        self.scan_pub.publish(scan)

    def publish_odom(self):
        odom = Odometry()
        odom.header.stamp = self.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        odom.pose.pose.position.x = self.pose['x']
        odom.pose.pose.position.y = self.pose['y']
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self.pose['theta'] / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.pose['theta'] / 2.0)
        
        odom.pose.covariance = [0.01 if i == j else 0.0 for i in range(6) for j in range(6)]
        
        odom.twist.twist.linear.x = self.cmd_vel['v']
        odom.twist.twist.angular.z = self.cmd_vel['w']
        
        odom.twist.covariance = [0.01 if i == j else 0.0 for i in range(6) for j in range(6)]
        
        self.odom_pub.publish(odom)

    def check_collision(self, new_x, new_y):
        robot_radius = 0.35  
        for wall in self.walls:
            x1, y1 = wall[0]
            x2, y2 = wall[1]
            dx = x2 - x1
            dy = y2 - y1
            px = new_x - x1
            py = new_y - y1
            line_len_sq = dx*dx + dy*dy
            if line_len_sq == 0:
                closest_x, closest_y = x1, y1
            else:
                t = max(0.0, min(1.0, (px * dx + py * dy) / line_len_sq))
                closest_x = x1 + t * dx
                closest_y = y1 + t * dy
            dist_sq = (new_x - closest_x)**2 + (new_y - closest_y)**2
            if dist_sq < robot_radius**2:
                return True 
        return False

def main(args=None):
    rclpy.init(args=args)
    node = AmrSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()