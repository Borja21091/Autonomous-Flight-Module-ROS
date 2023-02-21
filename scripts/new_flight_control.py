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
        self.frame = data.header.frame_id

class SensorMount(object):
    def __init__(self):
        # Initialise sensors
        self.lSensor = RangeSensor('left/teraranger_evo')
        self.fSensor = RangeSensor('front/teraranger_evo')
        self.rSensor = RangeSensor('right/teraranger_evo')
        
        # Initialise 3D points detected by the sensors (reference frame 'sensor_mount')
        self.Points = np.zeros([3,3])
        
        # Transform Listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Publishers
        self.distance = rospy.Publisher('distance',Range,queue_size = 10)
        self.normal = rospy.Publisher('normal',Point,queue_size = 10)
        
    def calculate_distance(self):
        # Transform distances to 3D points (reference frame 'sensor_mount')
        lSensorPoint = self.transform_measurement(self.lSensor)
        fSensorPoint = self.transform_measurement(self.fSensor)
        rSensorPoint = self.transform_measurement(self.rSensor)
        
        self.Points = np.array([lSensorPoint, fSensorPoint, rSensorPoint])
        
    def transform_measurement(self,sensor):
        # Move sensor data to common reference frame "base_link"
        while not sensor.frame:
            rospy.loginfo('Sensor Frame: %s', sensor.frame)
            
        TFSensor = self.tfBuffer.lookup_transform(sensor.frame,'base_link', rospy.Time(0))
        # Extract transformation
        p = np.array([TFSensor.transform.translation.x, TFSensor.transform.translation.y, TFSensor.transform.translation.z])
        q = np.array([TFSensor.transform.rotation.x, TFSensor.transform.rotation.y,
                      TFSensor.transform.rotation.z, TFSensor.transform.rotation.w])
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
        self.v_max = 0.05
        self.w_max = 0.26 # ~15 degrees/s
        
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
        # rot = quaternion_matrix([w,x,y,z])
        rot = quaternion_matrix([x,y,z,w])
        # Don't need full 4x4 matrix, 3x3 is good
        rot = rot[0:3,0:3]

        return rot

    def constant_height(self):
        # Initialize velocity
        vel_out = np.array([0.0, 0.0, 0.0])
        
        return vel_out

class pathPlanner():
    def __init__(self):
        # Params
        self.loop_rate = rospy.Rate(10)
        self.vel_output = np.array([0.0, 0.0, 0.0])
        self.vel_output_global = np.array([0.0, 0.0, 0.0])
        self.rG = np.zeros([3,1])
        self.w_z = 0.0
        
        # Initialize drone object
        self.drone = Drone()
        
        # Surface plane parameters
        self.n = None
        self.D = None
        self.R = np.zeros([3,3]) # Rotation matrix of plane-SensorMount
        
        # Initialize Controllers
        # self.vPlanarCtrl = PID(0.01,0.1,0.025) # 2D velocity estimation + correction
        self.vPlanarCtrl = PID(0.05,0.1,0.1) # 2D velocity estimation + correction
        self.rotZCtrl = PID(0.1,0.1,0.0) # Z axis rotation control
        # self.vx3DCtrl = PID(-0.025,-0.1,0.0) # 3D forward velocity control
        self.vx3DCtrl = PID(-0.1,-0.1,0.0) # 3D forward velocity control
        self.refDist = 0.85 # 0.9 metres target distance drone - wall
        
        # Raster trajectory parameters
        self.radius = 0.15
        self.Nturns = 3.0
        self.sideLength = 0.6
        self.firstTime = True

        # Initialize trajectory and project
        self.traj = raster(self.radius,self.Nturns,self.sideLength)
        CV = 0.05
        self.proj = project_trajectory(self.traj,self.vPlanarCtrl.Kp,self.vPlanarCtrl.Kd,self.vPlanarCtrl.Ki,CV)
        
        # Define axes
        flag = True
        while flag:
            rot = self.drone.get_rotation()
            if not(np.array_equal(rot,np.identity(3))):
                flag = False
        axis1 = np.array([0.0, -1.0, 0.0]) # 'axis1' -> horizontal axis in 2D // np.array([local_x_axis_3D, local_y_axis_3D, local_z_axis_3D]) is it 'world' really? Or could ir be any other 3D axes?
        axis2 = np.array([0.0, 0.0, 1.0]) # 'axis2' -> vertical axis in 2D // np.array([local_x_axis_3D, local_y_axis_3D, local_z_axis_3D])

        # Generate trajectory
        self.proj.generate(axis1,axis2)
        
    def fitNormalVec(self,rot):
        p0, p1, p2 = self.drone.sensor_mount.Points
        # rospy.loginfo('Distance Points: %s', self.drone.sensor_mount.Points)
        # Apply drone rotation --> p_i referenced at vicon_world
        p0 = np.matmul(rot,p0)
        p1 = np.matmul(rot,p1)
        p2 = np.matmul(rot,p2)
        # Extract components
        x0, y0, z0 = p0
        x1, y1, z1 = p1
        x2, y2, z2 = p2
        # Vectors that form a plane
        ux, uy, uz = [x1-x0, y1-y0, z1-z0]
        vx, vy, vz = [x2-x0, y2-y0, z2-z0]
        # Calculate normal and D
        u_cross_v = np.array([[uy*vz-uz*vy], [uz*vx-ux*vz], [ux*vy-uy*vx]])
        point  = np.array(p0)
        normal = np.array(u_cross_v); normal = normal/np.linalg.norm(normal)
        d = -point.dot(normal)
        # Publish distance
        msgRange = Range()
        msgRange.range = d
        self.drone.sensor_mount.distance.publish(msgRange)
        # Publish normal vector
        msgNormal = Point()
        msgNormal.x = normal[0]
        msgNormal.y = normal[1]
        msgNormal.z = normal[2]
        self.drone.sensor_mount.normal.publish(msgNormal)
        
        return normal,d
        
    def projectVec2Plane(self,pos3D,plane_normal):
        # Project 3D position to plane given by normal vector
        projected_pos3D = pos3D - np.multiply(np.dot(np.transpose(pos3D),plane_normal),plane_normal) / np.linalg.norm(plane_normal)**2
        # 2D Position
        # drone.pos2D += np.array([-projected_pos3D[1], projected_pos3D[2]])
        
        return projected_pos3D
        
    def merge2Twist(self,msgTwist,vel,w):
        # Message header frame_id
        msgTwist.header.frame_id = 'vicon_world'
        msgTwist.header.stamp = rospy.Time.now()
        # Message linear velocities
        msgTwist.twist.linear.x = vel[0] # Positive value --> To the front on vicon global
        msgTwist.twist.linear.y = vel[1] # Positive value --> To the left on vicon global
        msgTwist.twist.linear.z = vel[2] # Positive value --> Up in vicon global
        # Message angular velocities
        # TODO probably need to change angular commands to compensate
        msgTwist.twist.angular.x = 0.0
        msgTwist.twist.angular.y = 0.0
        msgTwist.twist.angular.z = w
        
    def distance_control(self):
        # Horizontal distance
        distx = -self.D / self.n[0]
        # rospy.loginfo('Distance to wall: %s', distx)
        # Error distance
        e_dist = self.refDist - distx
        # Estimate velocity PID
        v_out = self.vx3DCtrl.Kp*e_dist
        
        return v_out
        
    def rotation_control(self):
        # Error Normal Vector
        e_normal = self.n[1] # Y component of normal vector --> ref = 0 to be perpendicular to surface
        # Estimate angular velocity PID
        w_out = self.rotZCtrl.Kp*e_normal
        
        return w_out        
        
    def start(self):
        pos_1 = np.zeros([3,1])
        while not rospy.is_shutdown():
            # Drone's global Position and Orientation
            flag = True
            while flag:
                pos = self.drone.get_position()
                if not(np.array_equal(pos,np.zeros([3,1]))):
                    flag = False
            rot = self.drone.get_rotation()
            
            # Ranging Sensor Measurements [m]
            self.drone.sensor_mount.calculate_distance()
            
            # Fit plane to Sensor points
            # rospy.loginfo('Rotation Matrix: %s', rot)
            self.n,self.D = self.fitNormalVec(rot)
            # rospy.loginfo('Plane normal: %s', self.n)
            
            # Project Drone 3D (base_link) position to 2D
            self.rG = pos - pos_1
            if self.firstTime:
                self.rG = np.zeros([3,1])
                self.firstTime = False
            # rospy.loginfo('Normal vec: %s', self.n)
            # rospy.loginfo('Rot Matrix: %s', rot)
            # rospy.loginfo('Rotation Matrix: %s', rot)
            r_drone = np.matmul(np.transpose(rot),self.rG)
            # rospy.loginfo('r_drone: %s', r_drone)
            rProj = self.projectVec2Plane(r_drone,self.n)
            # rospy.loginfo('Projected r_drone: %s', rProj)
            pos_1 = np.copy(pos)
            
            # Project velocities 2D --> 3D local
            # Linear Velocity
            velParam = np.empty([3,1])
            velParam = self.proj.projVel(rProj)
            self.vel_output[0] = self.distance_control()
            self.vel_output[1] = velParam[1,0]
            self.vel_output[2] = velParam[2,0]
            # rospy.loginfo('Target Local Vel: %s', self.vel_output)
            # cmd_vel commands NEED to be in Vicon Global frame
            self.vel_output_global = np.matmul(rot,self.vel_output)
            self.vel_output_global = self.drone.v_max*self.vel_output_global / np.linalg.norm(self.vel_output_global)
            # rospy.loginfo('Target Global Vel: %s', self.vel_output_global)
            # Angular Velocity
            self.w_z = self.rotation_control()
            self.w_z = min(max(-self.drone.w_max, self.w_z),self.drone.w_max)
            # rospy.loginfo('Rotation target: %s', self.w_z)
            if self.proj.t_i > self.proj.t_end:
                self.vel_output_global[0] = 0.0
                self.vel_output_global[1] = 0.0
                self.vel_output_global[2] = 0.0
                self.w_z = 0.0
                # rospy.loginfo("Stopping Motion")
                    
            # Merge local velocities to Twist ROS msg
            self.merge2Twist(self.drone.vel,self.vel_output_global,-self.w_z)
            
            # Publish msg to /mavros/setpoint_velocity/cmd_vel
            self.drone.pub.publish(self.drone.vel)

if __name__ == '__main__':
    try:
        # Change drone mode to GUIDED
        bashCommand = "rosrun mavros mavsys mode -c GUIDED"
        os.system(bashCommand)
        # Init ROS Node
        rospy.init_node('follow_path', anonymous = True)
        rospy.loginfo('Starting follow_path Node...')
        controller = pathPlanner()
        controller.start()
    except rospy.ROSInterruptException:
        pass
