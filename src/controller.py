#!/usr/bin/env python

# imports
import rospy

from mavros_msgs.srv import (
    SetMode, SetModeRequest,
    CommandBool, CommandBoolRequest,
    CommandTOL, CommandTOLRequest
)

from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from sensor_msgs.point_cloud2 import PointCloud2

import ros_numpy

from cv_bridge import CvBridge

from enum import Enum
import math
import cv2
import numpy as np

# UAV states
class States(Enum):
    TAKEOFF = 1
    FLYING = 2
    MARKER_DET = 3
    LANDING = 4
    LANDED = 5

# Main controller class
class Controller():
    def __init__(self):
        # linear velocity and angular velocity
        self.linear_vel = 0.5
        self.angular_vel = 0.2

        rospy.init_node('controller', anonymous=True)

        # wait for services
        rospy.wait_for_service('/mavros/set_mode')
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/cmd/takeoff')
        rospy.wait_for_service('/mavros/cmd/land')

        # services
        self.mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_srv = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.land_srv = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

        # subscribers
        self.forward_pc = rospy.Subscriber('/depth_camera/depth/points', PointCloud2, self.pc_callback)
        self.downward_rgb_cam = rospy.Subscriber('/camera/color/image_raw', Image, self.d_rgb_callback)
        # self.downward_rgb_cam = rospy.Subscriber('/webcam/image_raw', Image, self.d_rgb_callback)
        self.pose_pub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        self.bridge = CvBridge()

        # publishers
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=5)
        self.aruco_pub = rospy.Publisher('/aruco/message', String, queue_size=5)

        # variables
        self.state = States.LANDED
        self.altitude = 0
        self.yaw = 0
    
    def takeoff(self):
        self.state = States.TAKEOFF

        # set mode to guided
        req = SetModeRequest()
        req.custom_mode = 'guided'
        self.mode_srv(req)

        # arm the drone
        req = CommandBoolRequest()
        req.value = True
        self.arm_srv(req)

        # take off to a height of 3m
        req = CommandTOLRequest()
        req.altitude = 3.0
        self.takeoff_srv(req)

        # it takes approx 5s to takeoff
        rospy.sleep(5)

        self.state = States.FLYING
    
    def land(self):
        self.state = States.LANDING
        # land
        self.land_srv(CommandTOLRequest())
        self.state = States.LANDED
    
    def step(self, actions):
        # combine several actions and form a single Twist message
        # this Twist message encodes the velocity setpoint

        msg = Twist()

        # x axis positive forward
        msg.linear.x += actions['x'] * self.linear_vel * math.cos(self.yaw)
        msg.linear.y += actions['x'] * self.linear_vel * math.sin(self.yaw)

        # y axis positive left
        msg.linear.x += actions['y'] * -self.linear_vel * math.sin(self.yaw)
        msg.linear.y += actions['y'] * self.linear_vel * math.cos(self.yaw)

        # z axis positive up
        msg.linear.z = actions['z'] * self.linear_vel

        # rotation positive left (anticlockwise)
        msg.angular.z = actions['r'] * self.angular_vel
        
        self.setpoint_pub.publish(msg)

    def pc_callback(self, msg):
        # main code for navigation. Uses potential based navigation

        if self.state != States.FLYING:
            return

        # convert the point cloud to a numpy array
        pc = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg)
        
        # sample only 1000 points to improve computation speed
        if pc.shape[0] > 1000:
            indices = np.random.choice(pc.shape[0], size=500)
            pc = pc[indices]

        # based on the potential function calculate a force
        d = np.linalg.norm(pc, axis=1).reshape((-1, 1))

        forces = 0.1 * pc / (1e-6 + d) **2
        force = np.sum(forces, axis=0)

        # add large forward force to incentivize the UAV to move forward
        force[2] -= 0.05 * pc.shape[0]

        # # add virtual force to not let the drone exceed 5m altitude
        force[1] -= 0.001 * pc.shape[0] / (1e-6 + abs(self.altitude - 5))

        # convert the forces to velocity setpoints
        actions = {'x': 0, 'y': 0, 'z': 0, 'r': 0}

        actions['x'] = np.clip(-force[2]/10, -1, 2)
        actions['z'] = np.clip(force[1], -2, 2)
        actions['r'] = np.clip(force[0], -1, 1)
        
        # apply the action
        self.step(actions)

        # uncomment this section for a visualization

        # visualization
        img = np.ones((480, 480))

        x_coords = 240 + pc[:, 0] * 480/10
        y_coords = 480 - pc[:, 2] * 480/10

        x_coords = x_coords.astype(np.int)
        y_coords = y_coords.astype(np.int)

        x_coords = np.clip(x_coords, 0, 479)
        y_coords = np.clip(y_coords, 0, 479)

        img[y_coords, x_coords] = 0.0
        
        cv2.imshow('img', img)
        cv2.waitKey(1)
    
    def d_rgb_callback(self, msg):
        # publish aruco messages depending on state
        if self.state == States.MARKER_DET or self.state == States.LANDING or self.state == States.LANDED:
            self.aruco_pub.publish('Marker ID: 0, Landed')
        else:
            self.aruco_pub.publish('Marker ID: none, looking for marker')

        if self.state != States.FLYING and self.state != States.MARKER_DET:
            return

        # convert to opencv image
        img = self.bridge.imgmsg_to_cv2(msg)

        # convert the image colour
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

        # detect the aruco markers
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
        arucoParams = cv2.aruco.DetectorParameters_create()

        corners, ids, rejected = cv2.aruco.detectMarkers(img, arucoDict, parameters=arucoParams)

        idx = None

        # if detect marker id 0 among all the detected markers
        if ids is not None:
            for i, id in enumerate(ids):
                if id == 0:
                    idx = i
        
        # if marker id 0 is found proceed further
        if idx is not None:
            if self.state == States.FLYING:
                self.state = States.MARKER_DET
                self.forward_pc.unregister()
                print('marker detected')

            h, w, _ = img.shape

            # find the mean coordinate of the markers
            target = np.mean(np.squeeze(corners[idx]), axis=0)

            # fly towards the marker with virtual forces
            target[0] -= w/2
            target[1] -= h/2

            actions = {'x': 0, 'y': 0, 'z': 0, 'r': 0}

            actions['x'] = np.clip(-target[1]/150, -1, 1)
            actions['y'] = np.clip(-target[0]/150, -1, 1)
            
            # apply the action
            self.step(actions)

            # if the marker is in the center of the image, proceed to land
            if abs(target[0]) < 10 and abs(target[1]) < 10:
                self.land()

    def pose_callback(self, msg):
        self.altitude = msg.pose.position.z
        self.yaw = 2*math.atan2(msg.pose.orientation.z, msg.pose.orientation.w)

if __name__ == '__main__':
    c = Controller()
    print('Taking off')
    c.takeoff()
    print('Input Commands Now')
    # to pause execution, press a key to land
    raw_input()
    print('Landing')
    c.land()