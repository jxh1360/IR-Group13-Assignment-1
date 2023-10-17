from geometry_msgs.msg import Pose, PoseArray, Quaternion
from . pf_base import PFLocaliserBase
import math
import rospy

from . util import rotateQuaternion, getHeading
import random

from time import time


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # ----- Set motion model parameters
        self.ODOM_ROTATION_NOISE = 0 	# Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 0 	# Odometry x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 0 		# Odometry y axis (side-side) noise
 
        # ----- Sensor model parameters
        self.NUMBER_PREDICTED_READINGS = 20     # Number of readings to predict

        # ----- Number of particles
        self.NUMBER_OF_PARTICLES = 100
        self.POSE_ESTIMATION_CONST = 10
        
       
    def initialise_particle_cloud(self, initialpose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        rospy.loginfo("initial particle cloud setting started")
        InitialParticleCloud = PoseArray()

        for p in range(self.NUMBER_OF_PARTICLES):

            rndangle = random.normalvariate(0,1/3)    # keep adjustable based on the number of paricles
            rnddistx = random.normalvariate(0,1)           # keep adjustable based on the number of paricles
            rnddisty = random.normalvariate(0,1)           # keep adjustable based on the number of paricles
            part_pose = Pose()

            part_pose.position.x = (initialpose.pose.pose.position.x + rnddistx)
            part_pose.position.y = (initialpose.pose.pose.position.y + rnddisty)
            part_pose.position.z = initialpose.pose.pose.position.z
            part_pose.orientation = rotateQuaternion(initialpose.pose.pose.orientation,rndangle)

            InitialParticleCloud.poses.append(part_pose)
        
        return InitialParticleCloud
 
    
    def update_particle_cloud(self, scan):
        """
        This should use the supplied laser scan to update the current
        particle cloud. i.e. self.particlecloud should be updated.
        
        :Args:
            | scan (sensor_msgs.msg.LaserScan): laser scan to use for update

         """
        rospy.loginfo("updating particle cloud")

        # filtering using sensor scan data
        # finding weights associated with each particle
        weight_arr = []
        weight_sum = 0
        for p in range(self.NUMBER_OF_PARTICLES):
            part_weight = self.sensor_model.get_weight(scan, self.particlecloud.poses[p])
            weight_sum = weight_sum + part_weight
            weight_arr.append(part_weight)
        
        # normalize weights
        for p in range(self.NUMBER_OF_PARTICLES):
            weight_arr[p] = weight_arr[p] / weight_sum
        
        # running resampling step(stochastic universal sampling)
        # coming up with a new particle distribution
        cumulative_wt = []
        cumulative_wt.append(weight_arr[0])
        for i in range(1,self.NUMBER_OF_PARTICLES):
            cumulative_wt_local = cumulative_wt[i-1] + weight_arr[i]
            cumulative_wt.append(cumulative_wt_local)
        
        threshold_arr = []
        threshold_init = 1 / (self.NUMBER_OF_PARTICLES * 2)
        threshold_arr.append(threshold_init)
        i = 0
        new_cloud = PoseArray()
        for j in range(self.NUMBER_OF_PARTICLES):
            while threshold_arr[j] > cumulative_wt[i] and i + 1 < self.NUMBER_OF_PARTICLES:
                i = i + 1
            new_cloud.poses.append(self.particlecloud.poses[i])
            threshold_new = threshold_arr[j] + 1 / self.NUMBER_OF_PARTICLES
            threshold_arr.append(threshold_new)
        
        # replacing old particle cloud with the new one with added noise
        for p in range(self.NUMBER_OF_PARTICLES):
            rndangle = random.normalvariate(0,1/3)
            rnddistx = random.normalvariate(0,0.05)
            rnddisty = random.normalvariate(0,0.05)

            self.particlecloud.poses[p].position.x = new_cloud.poses[p].position.x + rnddistx
            self.particlecloud.poses[p].position.y = new_cloud.poses[p].position.y + rnddisty
            self.particlecloud.poses[p].orientation = rotateQuaternion(new_cloud.poses[p].orientation,rndangle)


    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """
        rospy.loginfo("estimating pose")
        # choose every 5th particle in the particle cloud
        posx = 0
        posy = 0
        quatx = 0
        quaty = 0
        quatz = 0
        quatw = 0
        particle_no = 0

        for p in range(0,self.NUMBER_OF_PARTICLES,self.POSE_ESTIMATION_CONST):
        #for p in range(0,self.NUMBER_OF_PARTICLES):
            particle_no = particle_no + 1
            posx = posx + self.particlecloud.poses[p].position.x
            posy = posy + self.particlecloud.poses[p].position.y
            quatx = quatx + self.particlecloud.poses[p].orientation.x
            quaty = quaty + self.particlecloud.poses[p].orientation.y
            quatz = quatz + self.particlecloud.poses[p].orientation.z
            quatw = quatw + self.particlecloud.poses[p].orientation.w
        
        posx = posx / particle_no
        posy = posy / particle_no
        quatx = quatx / particle_no
        quaty = quaty / particle_no
        quatz = quatz / particle_no
        quatw = quatw / particle_no

        est_pose = Pose()
        est_pose.position.x = posx
        est_pose.position.y = posy
        est_pose.position.z = self.estimatedpose.pose.pose.position.z
        est_pose.orientation = Quaternion(quatx,quaty,quatz,quatw)

        return est_pose
