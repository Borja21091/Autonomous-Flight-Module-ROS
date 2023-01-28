#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Point
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Range
from std_msgs.msg import String

from project_trajectory import project_trajectory, raster
import numpy as np
import os

from tf.transformations import quaternion_matrix
from tf.transformations import translation_matrix
import tf2_ros

class RangeSensor(object):
    def __init__(self,sSensorTopic):
        # Params
        self.frame = ""
        self.point = Point()
        
        # Subscriber
        rospy.Subscriber(sSensorTopic,Range,self.distance_callback)

    def distance_callback(self,data):
        self.point.x = data.range
        self.point.y = 0
        self.point.z = 0
        self.frame = data.frame_id

class SensorMount(object):
    def __init__(self):
        # Initialise sensors
        self.lSensor = RangeSensor('left_sensor/teraranger_evo')
        self.fSensor = RangeSensor('front_sensor/teraranger_evo')
        self.rSensor = RangeSensor('right_sensor/teraranger_evo')
        
        # Initialise 3D points detected by the sensors (reference frame 'sensor_mount')
        self.Points = np.zeros([3,3])
        
        # Transform Listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
    def calculate_distance(self):
        # Transform distances to 3D points (reference frame 'sensor_mount')
        lSensorPoint = self.transform_measurement(self.lSensor)
        fSensorPoint = self.transform_measurement(self.fSensor)
        rSensorPoint = self.transform_measurement(self.rSensor)
        
        self.Points = np.array([lSensorPoint, fSensorPoint, rSensorPoint])
        
        
    def transform_measurement(self,sensor):
        # Move sensor data to common reference frame "sensorMount"
        TFSensor = self.tfBuffer.lookup_transform(sensor.frame, 'sensor_mount', rospy.Time(0))
        # Extract transformation
        p = np.array([TFSensor.position.x, TFSensor.position.y, TFSensor.position.z])
        q = np.array([TFSensor.orientation.x, TFSensor.orientation.y,
                      TFSensor.orientation.z, TFSensor.orientation.w])
        r = quaternion_matrix(q)
        r = r[0:3,0:3]
        # Transform 3D point
        point = np.array([sensor.point.x, sensor.point.y, sensor.point.z])
        tPoint = np.matmul(r,point) + p
        
        return tPoint

class PID(object):
    def __init__(self,kp=0.0,ki=0.0,kd=0.0):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd

    def compute(self,error):
        self.out = self.Kp*error.pp +  self.Ki*error.i + self.Kd*error.d

class Drone(object):
    def __init__(self):
        # Params
        self.loop_rate = rospy.Rate(10)
        self.state = TransformStamped()
        self.vel = TwistStamped()
        self.pos2D = np.zeros([2,1])

        # Max speeds [m/s] & [rad/s]
        self.v_max = 0.1
        self.w_max = 0.26 # ~15 degrees

        # PID Controller
        self.pid = PID(0.025,0.1,0.025)
        
        # Sensor Mount
        self.sensor_mount = SensorMount()

        # Publisher
        self.pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel',TwistStamped,queue_size=10)

        # Subscriber
        rospy.Subscriber('vicon/SUI_Endurance/SUI_Endurance',TransformStamped,self.move_callback)

    def move_callback(self,data):
        # Store values
        self.state = data

    def get_position(self):
        # Initialize position variable
        pos = np.empty([3,1])
        # Arrange into array
        pos[0,0] = self.state.transform.translation.x
        pos[1,0] = self.state.transform.translation.y
        pos[2,0] = self.state.transform.translation.z
        
        return pos

    def get_rotation(self):
        # Parse rotation data (quaternion)
        x = self.state.transform.rotation.x
        y = self.state.transform.rotation.y
        z = self.state.transform.rotation.z
        w = self.state.transform.rotation.w
        # Transform quaternion to matrix
        rot = quaternion_matrix([w,x,y,z])
        # Don't need full 4x4 matrix, 3x3 is good
        rot = rot[0:3,0:3]

        return rot

    def constant_height(self):
        # Initialize velocity
        vel_out = np.array([0.0, 0.0, 0.0])
        
        return vel_out

class flight_control(object):
    def __init__(self):
        # Params
        self.loop_rate = rospy.Rate(10)
        
        # Publisher
        self.pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=1)

        # Initialize drone object
        self.drone = Drone()
        
        # Raster trajectory parameters
        self.radius = 0.2
        self.Nturns = 3.0
        self.sideLength = 0.4
        self.firstTime = True

        # Initialize trajectory and project
        self.traj = raster(self.radius,self.Nturns,self.sideLength)
        CV = 0.05
        self.proj = project_trajectory(self.traj,self.drone.pid.Kp,self.drone.pid.Kd,self.drone.pid.Ki,CV)
        
        # Define axes
        flag = True
        while flag:
            rot = self.drone.get_rotation()
            if not(np.array_equal(rot,np.identity(3))):
                flag = False
        axis1 = np.array([1.0, 0.0, 0.0]) # 'axis1' -> horizontal axis in 2D // np.array([world_x_axis_3D, world_y_axis_3D, world_z_axis_3D]) is it 'world' really? Or could ir be any other 3D axes?
        axis2 = np.array([0.0, 0.0, 1.0]) # 'axis2' -> vertical axis in 2D // np.array([world_x_axis_3D, world_y_axis_3D, world_z_axis_3D])

        # Generate trajectory
        self.proj.generate(axis1,axis2)

    def flu2frd(self,vel_in):
        # Initialize vector
        vel_out = np.empty([3,1])
        # Transform
        vel_out[0] = vel_in[0]
        vel_out[1] = -vel_in[1]
        vel_out[2] = -vel_in[2]

        return vel_out

    def merge2Twist(self,msgTwist,vel):
        # Message header frame_id
        msgTwist.header.frame_id = 'vicon/world'
        msgTwist.header.stamp = rospy.Time.now()
        # Message linear velocities
        msgTwist.twist.linear.x = vel[0] # Positive value --> To the front on vicon global
        msgTwist.twist.linear.y = vel[1] # Positive value --> To the left on vicon global
        msgTwist.twist.linear.z = vel[2] # Positive value --> Up in vicon global
        # Message angular velocities
        # TODO probably need to change angular commands to compensate
        msgTwist.twist.angular.x = 0.0
        msgTwist.twist.angular.y = 0.0
        msgTwist.twist.angular.z = 0.0

        rospy.loginfo('Speed Commands')
        rospy.loginfo(msgTwist)

    def broadcastTF(self,pos):
        t = TransformStamped()
        t.header.frame_id = "vicon/world"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "trajectory0"
        t.transform.translation.x = pos[0,0]
        t.transform.translation.y = pos[1,0]
        t.transform.translation.z = pos[2,0]

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tfm = TFMessage([t])
    
    def start(self):
        while not rospy.is_shutdown():        
            # Get drone global position & orientation
            flag = True
            while flag:
                pos = self.drone.get_position()
                if not(np.array_equal(pos,np.zeros([3,1]))):
                    flag = False
            rot = self.drone.get_rotation()
            
            # Fit plane to Sensor points
            self.n,self.D = self.fitNormalVec()

            # Publish 2D trajectory to /parametric_curve
            self.traj.publish_to_plot_curve(self.proj.inv_projection_matrix,rot)

            # Keep drone at constant height
            self.vel_output = self.drone.constant_height()

            # Project velocities 2D --> 3D local
            if self.firstTime:
                pos0 = pos
                self.proj.projVelFirstTime(pos0)
                self.firstTime = False

            else:
                velParam = np.empty([3,1])
                velParam = self.proj.projVel(pos,rot)
                # rospy.loginfo("Projecting 2D velocity to 3D local space")
                self.vel_output[0] = velParam[0,0]
                self.vel_output[1] = velParam[1,0]
                self.vel_output[2] = velParam[2,0]
                if self.proj.t_i > self.proj.t_end:
                    self.vel_output[0] = 0.0
                    self.vel_output[1] = 0.0
                    self.vel_output[2] = 0.0
                    rospy.loginfo("Stopping Motion")

            # Set fixed reference frame in 3D space
            self.broadcastTF(pos0)

            # Merge local velocities to Twist ROS msg
            self.merge2Twist(self.drone.vel,self.vel_output)

            # Publish msg to /mavros/setpoint_velocity/cmd_vel
            # rospy.loginfo(self.drone.vel)
            self.drone.pub.publish(self.drone.vel)
            
            # Publish trajectory0
            self.pub_tf.publish(self.tfm)
            
            self.loop_rate.sleep()