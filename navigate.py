"""ROS-only mission node. Receives the target ID and finds the start in camera frames."""
import argparse
import json
import math

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Range
from std_msgs.msg import String

from field import ARUCO_SIZE, CAMERA_X, HFOV, NODES, POSES, TAG_DISTANCE, route
from vision import detect


def angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9


class Courier(Node):
    def __init__(self, goal, camera, sonar, speed, fov):
        super().__init__('turtle_courier')
        self.goal, self.speed, self.fov = goal, speed, fov
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status = self.create_publisher(String, '/courier/result', 10)
        self.create_subscription(Image, camera, self.camera, qos_profile_sensor_data)
        self.create_subscription(Range, sonar, self.range, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.drive, qos_profile_sensor_data)
        self.sonar = None
        self.start_min_area = math.inf
        self.tags = {}
        self.start_time = self.start = self.origin = None
        self.path = []
        self.closed = set()
        self.reversing = self.finished = False
        self.start_lock = None

    def camera(self, msg):
        channels = {'mono8': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}
        n = channels.get(msg.encoding)
        if n is None or msg.step < msg.width*n or len(msg.data) != msg.height*msg.step:
            return
        pixels = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        pixels = pixels[:, :msg.width*n].reshape(msg.height, msg.width, n)
        gray = pixels if n == 1 else cv2.cvtColor(
            pixels, cv2.COLOR_RGB2GRAY if msg.encoding.startswith('rgb') else cv2.COLOR_BGR2GRAY)
        corners, ids = detect(gray)
        focal = msg.width / (2 * math.tan(self.fov / 2))
        self.start_min_area = (focal*ARUCO_SIZE/.39)**2
        self.tags = {} if ids is None else {
            int(i): (stamp(msg), math.atan2(msg.width/2 - p[0,:,0].mean(), focal),
                     cv2.contourArea(p))
            for p, i in zip(corners, ids.flat) if i in POSES
        }

    def range(self, msg):
        if (math.isfinite(msg.range) and msg.range >= 0 or msg.range == math.inf) and msg.max_range > 0:
            self.sonar = (stamp(msg), msg.range)

    def send(self, v=0., w=0.):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = max(-1.9, min(1.9, w))
        self.cmd.publish(msg)

    def finish(self, ok, reason):
        self.send()
        self.finished = True
        result = dict(navigation_ok=ok, reason=reason, start_detected=self.start,
                      goal=self.goal, sonar_replans=len(self.closed))
        self.get_logger().info(json.dumps(result))
        self.status.publish(String(data=json.dumps(result)))

    def drive(self, msg):
        t = stamp(msg)
        if self.start_time is None:
            self.start_time = t
        if self.finished:
            self.send()
            return
        if t - self.start_time > 105:
            self.finish(False, 'mission timeout')
            return
        if self.sonar is None or abs(t-self.sonar[0]) > .5:
            self.send()
            return

        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = 2*math.atan2(q.z, q.w)
        if self.origin is None:
            # shortcut: this course starts centred in a bay, so the nearest sign anchors x/y.
            candidates = [(area, i, bearing, seen) for i, (seen, bearing, area) in self.tags.items()
                          if abs(t-seen) < .5 and abs(bearing) < .36 and area > self.start_min_area]
            if self.start_lock is None and candidates:
                self.start_lock = t
            if self.start_lock is not None:
                # Use a frame taken after stopping, so its bearing matches the current odometry.
                candidates = [c for c in candidates if c[3] >= self.start_lock + .32]
                if not candidates and t-self.start_lock < .85:
                    self.send()
                    return
                if not candidates:
                    self.start_lock = None
            if not candidates:
                self.send(w=.8 if t-self.start_time > .45 else 0.)
                return
            _, self.start, bearing, _ = max(candidates)
            gx, gy, heading = POSES[self.start]
            # Correct the bearing for the camera mounted ahead of the turning axis.
            camera_ratio = CAMERA_X / TAG_DISTANCE[self.start]
            delta = bearing - math.asin(camera_ratio*math.sin(bearing))
            self.origin = (gx, gy, heading - delta - yaw, p.x, p.y)
            self.path = route(self.start, self.goal, self.closed)
            self.get_logger().info(f'Camera found start ArUco {self.start}. Route: {self.path}')

        gx, gy, rotation, ox, oy = self.origin
        x = gx + math.cos(rotation)*(p.x-ox) - math.sin(rotation)*(p.y-oy)
        y = gy + math.sin(rotation)*(p.x-ox) + math.cos(rotation)*(p.y-oy)
        yaw = angle(yaw+rotation)

        if not self.path:
            self.finish(False, 'no open road to goal')
            return
        if len(self.path) == 1:
            seen, bearing, area = self.tags.get(self.goal, (-1., 0., 0.))
            if abs(t-seen) < .45 and abs(bearing) < .24 and area > 500:
                self.finish(True, 'target ArUco confirmed in bay')
                return
            if abs(t-seen) < .45 and area > 500:
                self.send(w=2.8*bearing)
                return
            error = angle(POSES[self.goal][2]-yaw)
            self.send(w=2.5*error if abs(error) > .05 else 0.)
            return

        a, b = self.path[:2]
        tx, ty = NODES[a if self.reversing else b]
        distance = math.hypot(tx-x, ty-y)
        if distance < .027:
            if self.reversing:
                self.path = route(a, self.goal, self.closed)
                self.reversing = False
                self.get_logger().info(f'Alternate route: {self.path}')
            else:
                self.path.pop(0)
            self.send()
            return

        error = angle(math.atan2(ty-y, tx-x)-yaw-(math.pi if self.reversing else 0.))
        if abs(error) > .16:
            self.send(w=3.4*error)
        elif not self.reversing and self.sonar[1] < (.12 if b in POSES else .19):
            self.closed.add(frozenset((a, b)))
            self.reversing = True
            self.get_logger().info(f'Sonar {self.sonar[1]:.2f} m blocked {a} -> {b}')
            self.send()
        else:
            speed = min(self.speed, max(.055, 1.5*distance))
            self.send(-min(.12, speed) if self.reversing else speed, 3.0*error)


def main():
    cli = argparse.ArgumentParser()
    cli.add_argument('--goal', type=int, choices=POSES, required=True)
    cli.add_argument('--camera', default='/front_camera/image_raw')
    cli.add_argument('--sonar', default='/ultrasonic/front')
    cli.add_argument('--speed', type=float, default=.19)
    cli.add_argument('--fov', type=float, default=HFOV)
    args = cli.parse_args()
    if not 0 < args.speed <= .23 or not 0 < args.fov < math.pi:
        cli.error('speed must be in (0, .23], horizontal FOV must be in (0, pi)')
    rclpy.init()
    node = Courier(args.goal, args.camera, args.sonar, args.speed, args.fov)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.send()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
