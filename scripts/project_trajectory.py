#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped
import math
import numpy as np
from numpy.linalg import inv

class project_trajectory(object):
    def __init__(self,parametric_curve,KP,KD,KI,Cv):
        # Current value of parameter t
        self.t_i = 0.0
        self.t_end = 0.0

        self.projection = np.empty([2,1])
        self.trajectory = np.empty([2,1])

        # Trajectory class
        self.traj = parametric_curve
        # Inverse projection matrix
        self.inv_projection_matrix = np.empty([3,2])
        # Projection matrix
        self.projection_matrix = np.empty([2,3])
        # Reference point in projection plane
        self.x_proj_ref = np.empty([2,1])
        # Parameter for parametric trajectory
        self.t = 0.0
        self.KD = KD
        # Gains of the planar position correction
        self.KP = KP
        self.KI = KI
        # Maximum tangential velocity
        self.Cv = Cv
        
        # Publisher
        self.drone2D_pub = rospy.Publisher('drone2D', PointStamped, queue_size = 10)

    def generate(self,axis1,axis2):
        # Copy final value of parameter T
        self.t_end = self.traj.T
        # Get inverse projection matrix
        self.inv_projection_matrix[:,0] = axis1
        self.inv_projection_matrix[:,1] = axis2
        # Initialize parameter
        self.t = 0.01
        # Check if axis span a 2 dimensional space
        provMat = np.matmul(np.transpose(self.inv_projection_matrix),self.inv_projection_matrix)
        
        # DID NOT KNOW HOW TO TRANSCRIPT THIS!
        # p, l, u = lu(provMat) # LU decomposition
        # if(lu_decomp.isInvertible())

        # Get projection matrix
        self.projection_matrix = np.matmul(inv(provMat),np.transpose(self.inv_projection_matrix))
        
    def publish_dronePos_2D(self, projPos):
        msgPoint = PointStamped()
        msgPoint.header.frame_id = "drone2D"
        msgPoint.point.x = projPos[0,0]
        msgPoint.point.y = projPos[1,0]
        msgPoint.point.z = 0.0

        self.drone2D_pub.publish(msgPoint)

    def projVelFirstTime(self,pos0):
        self.x_proj_ref = np.matmul(self.projection_matrix,pos0)

    def projVel(self,pos):
        # Initialization
        # Number of iteration for gradient descent
        count = 0
        # Gradient of the function to be minimised
        DC = 1.0
        # Gradient descend rate
        alpha = 0.3

        # Project global drone prosition and subtract reference point (origin of parametric trajectory)
        xProj = np.matmul(self.projection_matrix,pos) - self.x_proj_ref
        
        # Publish 2D projected position
        self.publish_dronePos_2D(xProj)

        # Loop to find closest point in trajectory to real projected position
        xTraj = np.empty([2,1])
        dxTraj = np.empty([2,1])
        errorProj = np.empty([2,1])
        sErrorProj = np.zeros([2,1])

        while (abs(DC) > 0.001 and count < 50):
            count = count + 1
            # Get position for given parameter
            xTraj = self.traj.pos(self.t)
            # Get derivative for given t
            dxTraj = self.traj.dpos(self.t)
            # Distance between trajectory and real projected drone position
            errorProj[0] = xTraj[0] - xProj[0]
            errorProj[1] = xTraj[1] - xProj[1]
            # Derivative of the distance at t
            DC = np.matmul(np.transpose(errorProj),dxTraj)
            # Update trajectory parameter t according to gradient
            self.t -= alpha*DC
            if self.t < 0.0:
                # Make t always positive
                self.t = -self.t

        # Accumulation of position error
        sErrorProj[0] += errorProj[0]
        sErrorProj[1] += errorProj[1]
            
        # Update parameter t global
        self.t_i = self.t
        self.projection = xProj
        self.trajectory = xTraj

        # Compute planar velocity
        vProj = np.empty([2,1])
        vProj = self.KD*dxTraj + self.KP*errorProj + self.KI*sErrorProj

        # Transform planar velocity to Global space
        vGlobal = np.empty([3,1])
        vGlobal = np.matmul(self.inv_projection_matrix,vProj)
        
        # Norm of the planar velocity
        vGlobal_norm = np.linalg.norm(vGlobal)
        
        # Normalization to have constant tangential velocity of C_v
        if vGlobal_norm > self.Cv:
            vGlobal *= self.Cv/vGlobal_norm
        
        """# Transform global to local (vicon drone frame) velocity
        # vLocal = np.empty([2,1])
        vLocal = np.matmul(rot,vGlobal[0:3])
        # vLocal = np.matmul(np.transpose(rot),vGlobal[0:3]) # Original code

        # Norm of the planar velocity
        vLocal_norm = np.linalg.norm(vLocal)

        # Normalization to have constant tangential velocity of C_v
        if vLocal_norm > 0.0:
            vLocal *= self.Cv/vLocal_norm"""

        return vGlobal

class raster(object):
    def __init__(self,turnWidth,turns,sideLength):
        # Copy raster parameters
        self.turns_ = turns
        self.sideLength = sideLength
        self.turnWidth = turnWidth

        self.H = 0.5*self.sideLength # Horizontal length
        self.V = self.turnWidth # Vertical length
        self.Lc = self.sideLength + (0.5*math.pi - 1.0)*self.V #Length of a cycle
        self.SIG = np.array([-1, 1])

        # Max value of parameter
        self.T = self.turns_*self.Lc

        # Publisher
        self.curvature_pub = rospy.Publisher('parametric_curve', Path, queue_size = 10)
        
        # Publish 2D trajectory
        self.publish_projection_2D()
        
    def publish_projection_2D(self):
        t_raster = 0
        n = 0
        msgPath = Path()
        msgPath.header.frame_id = "trajectory0"
        while t_raster < self.T:
            t_raster = t_raster + 0.01
            point = self.pos(t_raster)
            # point = np.matmul(np.transpose(point),np.linalg.inv(rot))
            msgPoint = PoseStamped()
            msgPoint.header.frame_id = "trajectory0"
            msgPoint.pose.position.x = point[0,0]
            msgPoint.pose.position.y = point[1,0]
            msgPoint.pose.position.z = 0.0
            msgPoint.pose.orientation.x = 0
            msgPoint.pose.orientation.y = 0
            msgPoint.pose.orientation.z = 0
            msgPoint.pose.orientation.w = 1
            msgPath.poses.append(msgPoint)
            n = n + 1

        self.curvature_pub.publish(msgPath)

    def pos(self,t):
        # Avoid negative parameter
        if t < 0.0:
            t = -t

        # Number of cycles
        N_c = int(t/self.Lc)
        # Remaining length
        L_left = t - N_c*self.Lc
        ind = N_c%2
        # Sign of trajectory
        sig = self.SIG[ind]
        # Choose line of trajectory
        x = np.empty([2, 1])

        # Straight direction 1
        if L_left < (self.H - 0.5*self.V):
            x[0,0] = N_c*self.V
            x[1,0] = sig*L_left

        # Turning
        elif L_left < (self.H + 0.5*(math.pi - 1.0)*self.V):
            x[0,0] = (N_c + 0.5)*self.V + 0.5*self.V*math.sin(sig*(L_left - self.H + 0.5*self.V)*2.0/self.V - 0.5*math.pi)
            x[1,0] = sig*(self.H - 0.5*self.V) + 0.5*self.V*math.cos(sig*(L_left - self.H + 0.5*self.V)*2.0/self.V - 0.5*math.pi)

        # Straight direction 2
        else:
            x[0,0] = (N_c + 1)*self.V
            x[1,0] = sig*(2.0*self.H + (0.5*math.pi - 1.0)*self.V - L_left)

        return x
        
    def dpos(self,t):
        # Avoid negative parameter
        if t < 0.0:
            t = -t

        # Number of cycles
        N_c = int(t/self.Lc)
        # Remaining length
        L_left = t - N_c*self.Lc
        ind = N_c%2
        # Sign of trajectory
        sig = self.SIG[ind]
        # Choose line of trajectory
        dx = np.empty([2, 1])

        if L_left < (self.H - 0.5*self.V):
            dx[0,0] = 0.0
            dx[1,0] = sig

        elif L_left < (self.H + 0.5*(math.pi - 1.0)*self.V):
            dx[0,0] = sig*math.cos(sig*(L_left - self.H + 0.5*self.V)*2.0/self.V - 0.5*math.pi)
            dx[1,0] = -sig*math.sin(sig*(L_left - self.H + 0.5*self.V)*2.0/self.V - 0.5*math.pi)

        else:
            dx[0,0] = 0.0
            dx[1,0] = -sig
        
        return dx