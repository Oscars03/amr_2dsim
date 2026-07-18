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
import numpy as np
import xml.etree.ElementTree as ET
import logging

_DEFAULTS = {
    'kinematic_model': 'diff_drive',
    'wheel_base': 0.5,
    'robot_radius': 0.35,
    'laser_range_max': 12.0,
    'ticks_per_meter': 1000.0,
    'drive_axle_x': None,  # None = not declared in URDF (skip check)
}

def _get_float(elem, tag, default, warn_cb=None):
    """Parse a single float field from XML element; return default on any error."""
    child = elem.find(tag)
    if child is None or not (child.text or '').strip():
        return default
    try:
        return float(child.text.strip())
    except (ValueError, AttributeError) as e:
        if warn_cb:
            warn_cb(f"amr_sim_config: cannot parse <{tag}>: {e}, using default {default}")
        return default

def parse_sim_config(urdf_path: str) -> dict:
    """Parse <amr_sim_config> from URDF; returns dict with defaults for missing/invalid fields."""
    cfg = dict(_DEFAULTS)
    try:
        root = ET.parse(urdf_path).getroot()
        sim_cfg = root.find('amr_sim_config')
        if sim_cfg is None:
            return cfg
        # kinematic_model (string)
        km = sim_cfg.find('kinematic_model')
        if km is not None and (km.text or '').strip():
            cfg['kinematic_model'] = km.text.strip()
        # numeric fields — each isolated so one bad value doesn't block others
        for field in ('wheel_base', 'robot_radius', 'laser_range_max', 'ticks_per_meter'):
            cfg[field] = _get_float(sim_cfg, field, cfg[field])
        # drive_axle_x: optional; keep None if absent (means "skip convention check")
        ax_elem = sim_cfg.find('drive_axle_x')
        if ax_elem is not None and (ax_elem.text or '').strip():
            try:
                cfg['drive_axle_x'] = float(ax_elem.text.strip())
            except ValueError:
                pass
    except Exception as e:
        logging.warning(f"parse_sim_config: failed to read '{urdf_path}': {e}")
    return cfg


def _check_drive_axle_convention(root: ET.Element, declared_axle_x, warn_cb, info_cb) -> None:
    """
    Verify that joints declared as drive wheels actually sit at declared_axle_x
    in the base_link frame.  Issues a WARNING (not an error) if they don't so
    that misconfigured robots are caught early without breaking the sim.

    Detection heuristic (covers standard ROS naming):
      - joint type="continuous" whose child link name contains 'wheel'
        AND (joint name or child name) contains 'front' OR 'drive'
      - Fallback: if <ros2_control> has <command_interface name="velocity">
        joints — those are the actuated wheels.
    ponytail: heuristic covers 95% of real robots; exotic naming needs explicit
              drive_axle_x tag. Upgrade: add a <drive_joints> tag to amr_sim_config.
    """
    if declared_axle_x is None:
        return  # user did not declare drive_axle_x → skip silently

    # Collect candidate drive joint names from ros2_control (most reliable)
    ros2_drive_joints = set()
    for rc in root.findall('ros2_control'):
        for joint in rc.findall('joint'):
            if joint.find('command_interface[@name="velocity"]') is not None:
                ros2_drive_joints.add(joint.get('name', ''))

    offenders = []
    for joint in root.findall('joint'):
        if joint.get('type') != 'continuous':
            continue
        jname = joint.get('name', '')
        child = joint.find('child')
        cname = child.get('link', '') if child is not None else ''

        # Is this a drive wheel? ros2_control list wins; fallback to name heuristic
        is_drive = (
            jname in ros2_drive_joints
            or (
                'wheel' in cname
                and ('front' in jname or 'drive' in jname or 'front' in cname)
            )
        )
        if not is_drive:
            continue

        origin = joint.find('origin')
        if origin is None or not origin.get('xyz'):
            continue
        xyz = origin.get('xyz').split()
        if len(xyz) < 1:
            continue
        actual_x = float(xyz[0])
        if abs(actual_x - declared_axle_x) > 1e-4:
            offenders.append((jname, actual_x))

    if offenders:
        for jname, ax in offenders:
            warn_cb(
                f"URDF convention violation: joint '{jname}' drive wheel is at "
                f"x={ax:.4f} in base_link frame, but <drive_axle_x> declares "
                f"x={declared_axle_x:.4f}.  ICC will be offset by "
                f"{ax - declared_axle_x:.4f} m — odometry will drift."
            )
    else:
        info_cb(
            f"Drive-axle convention OK: all drive wheels at x={declared_axle_x:.4f} "
            "(matches <drive_axle_x> declaration)."
        )

class AmrSimulator(Node):
    def __init__(self):
        super().__init__('amr_2sim')
        
        self.declare_parameter('map_file', '')
        self.declare_parameter('urdf_file', '')
        
        map_file_path = self.get_parameter('map_file').value
        urdf_file_path = self.get_parameter('urdf_file').value
        
        # Load kinematic config from URDF (or defaults if tag absent)
        self.laser_offset_x = 0.0
        self.laser_offset_y = 0.0
        self.laser_frame_id = 'laser_link'

        if urdf_file_path:
            cfg = parse_sim_config(urdf_file_path)
            self.kinematic_model = cfg['kinematic_model']
            self.wheel_base      = cfg['wheel_base']
            self.robot_radius    = cfg['robot_radius']
            self.laser_range_max = cfg['laser_range_max']
            self.ticks_per_meter = cfg['ticks_per_meter']
            self.get_logger().info(
                f"Loaded config from URDF: model={self.kinematic_model} "
                f"wheel_base={self.wheel_base} robot_radius={self.robot_radius}"
            )
            # laser joint offset + drive-axle convention check
            try:
                root = ET.parse(urdf_file_path).getroot()
                for joint in root.findall('joint'):
                    child = joint.find('child')
                    if child is not None:
                        link_name = child.get('link', '')
                        if 'laser' in link_name or 'lidar' in link_name:
                            self.laser_frame_id = link_name
                            origin = joint.find('origin')
                            if origin is not None and origin.get('xyz'):
                                xyz = origin.get('xyz').split()
                                if len(xyz) >= 2:
                                    self.laser_offset_x = float(xyz[0])
                                    self.laser_offset_y = float(xyz[1])
                            self.get_logger().info(
                                f"Loaded laser config: frame={self.laser_frame_id}, "
                                f"offset=({self.laser_offset_x}, {self.laser_offset_y})"
                            )
                            break
                _check_drive_axle_convention(
                    root,
                    cfg['drive_axle_x'],
                    self.get_logger().warning,
                    self.get_logger().info,
                )
            except Exception as e:
                self.get_logger().error(f"Failed to load laser joint from URDF: {e}")
        else:
            self.kinematic_model = _DEFAULTS['kinematic_model']
            self.wheel_base      = _DEFAULTS['wheel_base']
            self.robot_radius    = _DEFAULTS['robot_radius']
            self.laser_range_max = _DEFAULTS['laser_range_max']
            self.ticks_per_meter = _DEFAULTS['ticks_per_meter']

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

        # Convert walls to numpy arrays for vectorized raycasting
        self.wall_x3 = np.array([w[0][0] for w in self.walls])
        self.wall_y3 = np.array([w[0][1] for w in self.walls])
        self.wall_x4 = np.array([w[1][0] for w in self.walls])
        self.wall_y4 = np.array([w[1][1] for w in self.walls])

        self.cmd_vel = {'vx': 0.0, 'vy': 0.0, 'w': 0.0}
        self.last_time = time.time()

        # ----------------------------------------------------
        # 1. เพิ่มตัวแปรสำหรับเก็บค่า Encoder สะสม (Cumulative Pulses)
        # ----------------------------------------------------
        self.total_pulse_left = 0.0
        self.total_pulse_right = 0.0

        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.encoder_pub = self.create_publisher(String, '/wheel/encoder', 10)
        self.sub_cmd = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        self.wheel_vel_pub = self.create_publisher(Twist, '/wheel/vel', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)

        self.timer = self.create_timer(0.05, self.timer_callback)
        
    def cmd_callback(self, msg):
        self.cmd_vel['vx'] = msg.linear.x
        self.cmd_vel['vy'] = msg.linear.y
        self.cmd_vel['w'] = msg.angular.z

    def timer_callback(self):
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0:
            dt = 0.05
            
        self.last_time = current_time

        vx = self.cmd_vel['vx']
        vy = self.cmd_vel['vy']
        w = self.cmd_vel['w']

        # Determine robot velocities based on kinematic model
        if self.kinematic_model == 'diff_drive':
            vy = 0.0 # Diff drive cannot move sideways
        elif self.kinematic_model == 'ackermann':
            vy = 0.0 # Car cannot move sideways
            # For a basic ackermann, w = vx / wheelbase * tan(steering_angle)
            # Here we just assume w is already constrained or derived, or we just use it directly.
        
        # Integrate Position (Global Frame) ด้วยสมการ Midpoint 
        mid_theta = self.pose['theta'] + (w * dt / 2.0)
        new_theta = self.pose['theta'] + (w * dt)
        
        # Rotate local vx, vy to global frame (ใช้มุมค่ากลางเพื่อความแม่นยำตอนเข้าโค้ง)
        v_global_x = vx * math.cos(mid_theta) - vy * math.sin(mid_theta)
        v_global_y = vx * math.sin(mid_theta) + vy * math.cos(mid_theta)

        new_x = self.pose['x'] + (v_global_x * dt)
        new_y = self.pose['y'] + (v_global_y * dt)

        if not self.check_collision(new_x, new_y):
            self.pose['x'] = new_x
            self.pose['y'] = new_y
        else:
            # Try sliding along X or Y
            if not self.check_collision(new_x, self.pose['y']):
                self.pose['x'] = new_x
                self.get_logger().warn("Collision! Sliding along X.", throttle_duration_sec=1.0)
            elif not self.check_collision(self.pose['x'], new_y):
                self.pose['y'] = new_y
                self.get_logger().warn("Collision! Sliding along Y.", throttle_duration_sec=1.0)
            else:
                self.get_logger().warn("Collision Detected! Stopped translation.", throttle_duration_sec=1.0)
            
        self.pose['theta'] = new_theta

        # ----------------------------------------------------
        # Simulate Fake Encoder (approximated for all models to a differential equivalent for visualization)
        # ----------------------------------------------------
        v_right = vx + (w * self.wheel_base / 2.0)
        v_left = vx - (w * self.wheel_base / 2.0)
        
        # Calculate pulse increments
        delta_pulse_right = v_right * dt * self.ticks_per_meter
        delta_pulse_left = v_left * dt * self.ticks_per_meter
        
        # Accumulate pulses
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
        
        den = (x - x2) * (self.wall_y3 - self.wall_y4) - (y - y2) * (self.wall_x3 - self.wall_x4)
        valid = np.abs(den) > 1e-6
        
        t = np.full(len(self.walls), np.inf)
        u = np.full(len(self.walls), np.inf)
        
        t[valid] = ((x - self.wall_x3[valid]) * (self.wall_y3[valid] - self.wall_y4[valid]) - (y - self.wall_y3[valid]) * (self.wall_x3[valid] - self.wall_x4[valid])) / den[valid]
        u[valid] = -((x - x2) * (y - self.wall_y3[valid]) - (y - y2) * (x - self.wall_x3[valid])) / den[valid]
        
        intersect = (t >= 0) & (t <= 1) & (u >= 0) & (u <= 1)
        if np.any(intersect):
            return float(np.min(t[intersect]) * range_max)
        return float('inf')

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
        msg.angular_velocity_covariance[0] = 0.0001
        msg.linear_acceleration_covariance[0] = 0.0001
        
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
        scan.header.frame_id = self.laser_frame_id
        scan.angle_min = 0.0
        scan.angle_max = 2 * math.pi
        scan.angle_increment = math.radians(1.0) 
        scan.range_min = 0.05
        scan.range_max = self.laser_range_max
        scan.scan_time = 0.05
        scan.time_increment = 0.05 / 360.0
        
        # ponytail: dynamic laser offset based on URDF to fix map swinging
        laser_x = self.pose['x'] + self.laser_offset_x * math.cos(self.pose['theta']) - self.laser_offset_y * math.sin(self.pose['theta'])
        laser_y = self.pose['y'] + self.laser_offset_x * math.sin(self.pose['theta']) + self.laser_offset_y * math.cos(self.pose['theta'])

        ranges = []
        intensities = []
        for i in range(360):
            laser_angle = self.pose['theta'] + (i * scan.angle_increment)
            dist = self.get_ray_intersection(laser_x, laser_y, laser_angle, scan.range_max)
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
        
        # 1. Pose Covariance (Global 'odom' frame)
        # ความคลาดเคลื่อนตำแหน่งโลก (Global) ต้องสะสมและเพิ่มขึ้นทั้งแกน X และ Y
        pose_cov = [0.0] * 36
        pose_cov[0]  = 0.01  # Global X
        pose_cov[7]  = 0.01  # Global Y (แก้ตรงนี้: ห้ามเป็น 1e-5 เด็ดขาด)
        pose_cov[14] = 1e-5  # Global Z
        pose_cov[21] = 1e-5  # Roll
        pose_cov[28] = 1e-5  # Pitch
        pose_cov[35] = 0.01  # Yaw
        odom.pose.covariance = pose_cov
        
        odom.twist.twist.linear.x = self.cmd_vel['vx']
        odom.twist.twist.linear.y = 0.0 if self.kinematic_model == 'diff_drive' else self.cmd_vel['vy']
        odom.twist.twist.angular.z = self.cmd_vel['w']
        
        # 2. Twist Covariance (Local 'base_link' frame)
        # ความเร็วในมุมมองหุ่น (Local) แบบ Diff-Drive ไม่ไถลข้าง แกน Y จึงเป็น 1e-5 ได้
        twist_cov = [0.0] * 36
        twist_cov[0]  = 0.01 if self.kinematic_model == 'diff_drive' else 0.001  # Local X (Forward)
        twist_cov[7]  = 1e-5 if self.kinematic_model == 'diff_drive' else 0.001  # Local Y (Lateral/Slip)
        twist_cov[14] = 1e-5 # Local Z
        twist_cov[21] = 1e-5 # Roll
        twist_cov[28] = 1e-5 # Pitch
        twist_cov[35] = 0.01 # Yaw
        odom.twist.covariance = twist_cov
        
        self.odom_pub.publish(odom)

    def check_collision(self, new_x, new_y):
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
            if dist_sq < self.robot_radius**2:
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