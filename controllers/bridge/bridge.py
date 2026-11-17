"""Webots devices -> ROS Humble. Ground truth stays in the referee only."""
import json
import math
import os
from pathlib import Path
import time

from controller import Supervisor
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, Range
from std_msgs.msg import String

from field import BLOCKER, BODY_RADIUS, BUILDINGS, HFOV, ISLAND_CUBES, FIELD, POSES, TRACK, WHEEL


class Bridge(Node):
    def __init__(self):
        super().__init__('turtlebro_webots')
        self.robot = Supervisor()
        self.body = self.robot.getSelf()
        self.dt = int(self.robot.getBasicTimeStep())
        self.camera = self.robot.getDevice('camera')
        self.camera.enable(5*self.dt)
        self.imu = self.robot.getDevice("imu")
        self.imu.enable(self.dt)
        self.initial_heading = None
        self.sonar = self.robot.getDevice('front sonar')
        self.sonar.enable(self.dt)
        self.motors = [self.robot.getDevice(f'{side} motor') for side in ('left', 'right')]
        self.encoders = [self.robot.getDevice(f'{side} encoder') for side in ('left', 'right')]
        for motor, encoder in zip(self.motors, self.encoders):
            motor.setPosition(float('inf'))
            motor.setVelocity(0.)
            encoder.enable(self.dt)
        self.odom = self.create_publisher(Odometry, '/odom', qos_profile_sensor_data)
        self.image = self.create_publisher(Image, '/front_camera/image_raw', qos_profile_sensor_data)
        self.range_pub = self.create_publisher(Range, '/ultrasonic/front', qos_profile_sensor_data)
        self.clock = self.create_publisher(Clock, '/clock', qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)
        self.create_subscription(String, '/courier/result', self.completion, 10)
        self.x = self.y = self.yaw = 0.
        self.prev = self.pending = None
        self.v = self.w = 0.
        self.command_at = -100.
        self.min_clearance = math.inf
        self.contact = False
        self.boxes = [*BUILDINGS, *((x,y,s,s) for x,y,s in ISLAND_CUBES)]
        if os.environ['COURSE_BLOCKED'] == '1':
            self.boxes.append(BLOCKER)

    def command(self, msg):
        self.v, self.w = msg.linear.x, msg.angular.z
        if not all(math.isfinite(x) for x in (self.v,self.w)):
            self.v = self.w = 0.
        self.v = max(-.23, min(.23, self.v))
        self.w = max(-1.9, min(1.9, self.w))
        self.command_at = self.robot.getTime()

    def completion(self, msg):
        if self.pending is None:
            self.pending = (self.robot.getTime()+.45, json.loads(msg.data))

    def sample_referee(self):
        x, y, _ = self.body.getPosition()
        clear = min(FIELD[0]/2-abs(x), FIELD[1]/2-abs(y))
        for bx, by, w, d in self.boxes:
            clear = min(clear, math.hypot(max(0.,abs(x-bx)-w/2), max(0.,abs(y-by)-d/2)))
        self.min_clearance = min(self.min_clearance, clear-BODY_RADIUS)
        self.contact |= any(p.point[2] > .018 for p in self.body.getContactPoints(True))

    def result(self, nav):
        start, goal = int(os.environ['COURSE_START']), int(os.environ['COURSE_GOAL'])
        x, y, _ = self.body.getPosition()
        error = math.dist((x,y), POSES[goal][:2])
        ok = nav['navigation_ok'] and nav['start_detected'] == start and nav['goal'] == goal
        ok = ok and not self.contact and self.min_clearance > 0 and error <= .080
        result = dict(nav, passed=ok, start_actual=start, target_error_m=round(error,4),
                      min_clearance_m=round(self.min_clearance,4), contact=self.contact,
                      seconds=round(self.robot.getTime(),2))
        path = Path(os.environ['COURSE_RESULT'])
        temporary = path.with_suffix('.tmp')
        temporary.write_text(json.dumps(result, indent=2) + '\n')
        temporary.replace(path)
        print('REFEREE', json.dumps(result), flush=True)
        if os.environ['COURSE_GUI'] != '1':
            self.robot.simulationQuit(0 if ok else 2)

    def run(self):
        width, height = self.camera.getWidth(), self.camera.getHeight()
        step = 0
        evaluated = False
        while self.robot.step(self.dt) != -1:
            t = self.robot.getTime()
            ns = round(t*1e9)
            stamp = Time(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000)
            self.clock.publish(Clock(clock=stamp))
            rclpy.spin_once(self, timeout_sec=0.)
            vl, vr = (0.,0.) if t-self.command_at > .45 else (
                self.v-self.w*TRACK/2, self.v+self.w*TRACK/2)
            for motor, v in zip(self.motors, (vl,vr)):
                motor.setVelocity(v/WHEEL)
            positions = tuple(e.getValue() for e in self.encoders)
            if self.prev is not None:
                dl, dr = ((now-prev)*WHEEL for now,prev in zip(positions,self.prev))
                heading = self.imu.getRollPitchYaw()[2]
                if self.initial_heading is None:
                    self.initial_heading = heading
                new_yaw = heading-self.initial_heading
                da = math.atan2(math.sin(new_yaw-self.yaw), math.cos(new_yaw-self.yaw))
                ds = (dl+dr)/2
                self.x += ds*math.cos(self.yaw+da/2)
                self.y += ds*math.sin(self.yaw+da/2)
                self.yaw = new_yaw
                odom = Odometry(child_frame_id='base_footprint')
                odom.header.stamp, odom.header.frame_id = stamp, 'odom'
                odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
                odom.pose.pose.orientation.z = math.sin(self.yaw/2)
                odom.pose.pose.orientation.w = math.cos(self.yaw/2)
                self.odom.publish(odom)
            self.prev = positions
            if step % 5 == 0 and self.camera.getImage():
                image = Image(width=width, height=height, step=width*4, encoding='bgra8',
                              data=self.camera.getImage())
                if step == 20 and os.getenv("COURSE_FRAME"):
                    self.camera.saveImage(os.environ["COURSE_FRAME"], 95)
                image.header.stamp, image.header.frame_id = stamp, 'front_camera_optical_frame'
                self.image.publish(image)
            distance = self.sonar.getValue()
            reading = Range(radiation_type=Range.ULTRASOUND, field_of_view=.24,
                            min_range=.02, max_range=1.5,
                            range=distance if distance < 1.499 else math.inf)
            reading.header.stamp, reading.header.frame_id = stamp, 'front_sonar'
            self.range_pub.publish(reading)
            self.sample_referee()
            if step == 50 and os.environ['COURSE_GUI'] == '1' and os.getenv('COURSE_SCREEN'):
                self.robot.exportImage(os.environ['COURSE_SCREEN'], 95)
            if step % 125 == 0 and os.getenv('COURSE_TRACE'):
                print('TRACE', round(t, 1), self.body.getPosition(), self.x, self.y,
                      self.yaw, distance, self.min_clearance, flush=True)
            if self.pending and t >= self.pending[0] and not evaluated:
                evaluated = True
                self.result(self.pending[1])
            if t > 112 and not evaluated:
                evaluated = True
                self.result(dict(navigation_ok=False, reason='no mission result',
                                 goal=None, start_detected=None))
            step += 1
            if os.environ['COURSE_GUI'] != '1':
                time.sleep(.002)


rclpy.init()
node = Bridge()
try:
    node.run()
finally:
    node.destroy_node()
    rclpy.try_shutdown()
