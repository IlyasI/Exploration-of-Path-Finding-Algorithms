#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from geometry_msgs.msg  import Pose2D
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from math import pow,atan2,sqrt,pi
from planned_path import PlannedPath
import time
import math
import numpy

# This class defines a possible base of what the robot controller
# could do.

class ControllerBase(object):

    def __init__(self, occupancyGrid):

        rospy.wait_for_message('/robot0/odom', Odometry)

        # Create the node, publishers and subscriber
        self.velocityPublisher = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=10)
        self.currentOdometrySubscriber = rospy.Subscriber('/robot0/odom', Odometry, self.odometryCallback)

        # Specification of accuracy. The first is the Euclidean
        # distance from the target within which the robot is assumed
        # to be there. The second is the angle. The latter is turned
        # into radians for ease of the controller.
        self.distanceErrorTolerance = rospy.get_param('distance_error_tolerance', 0.05)
        self.goalAngleErrorTolerance = math.radians(rospy.get_param('goal_angle_error_tolerance', 0.1))

        # Set the pose to an initial value to stop things crashing
        self.pose = Pose2D()

        # Store the occupany grid - used to transform from cell
        # coordinates to world driving coordinates.
        self.occupancyGrid = occupancyGrid
        
        # This is the rate at which we broadcast updates to the simulator in Hz.
        self.rate = rospy.Rate(10)
        self.start_time = rospy.get_rostime().secs
	self.total_run_time = 0

        self.prev_coord_diff = None

    # Get the pose of the robot. Store this in a Pose2D structure because
    # this is easy to use. Use radians for angles because these are used
    # inside the control system.
    def odometryCallback(self, odometry):
        odometryPose = odometry.pose.pose

        pose = Pose2D()

        position = odometryPose.position
        orientation = odometryPose.orientation
        
        pose.x = position.x
        pose.y = position.y
        pose.theta = 2 * atan2(orientation.z, orientation.w)
        self.pose = pose

    # Return the most up-to-date pose of the robot
    def getCurrentPose(self):
        return self.pose

    # Handle the logic of driving the robot to the next waypoint
    def driveToWaypoint(self, waypoint):
        raise NotImplementedError()

    # Handle the logic of rotating the robot to its final orientation
    def rotateToGoalOrientation(self, waypoint):
        raise NotImplementedError()

    # Drive to each waypoint in turn. Unfortunately we have to add
    # the planner drawer because we have to keep updating it to
    # make sure the graphics are redrawn properly.
    def drivePathToGoal(self, path, goalOrientation, plannerDrawer):
        self.plannerDrawer = plannerDrawer

        rospy.loginfo('Driving path to goal with ' + str(len(path.waypoints)) + ' waypoint(s)')
        self.start_time = rospy.get_rostime().secs
        # Drive to each waypoint in turn
        for waypointNumber in range(0, len(path.waypoints)):
            cell = path.waypoints[waypointNumber]
            print "waypoint number: "+str(waypointNumber)
            if waypointNumber+1 == len(path.waypoints):
                next_cell = None
                coord_diff = None
            else:
                next_cell = path.waypoints[waypointNumber+1]
                coord_diff = tuple(numpy.subtract(next_cell.coords,cell.coords))
            #check if next waypoint is in a line with the current waypoint
            #only drive to waypoints at the beginning and end of each straight line

            if (coord_diff != self.prev_coord_diff) and (self.prev_coord_diff!=None):
            	waypoint = self.occupancyGrid.getWorldCoordinatesFromCellCoordinates(cell.coords)
            	rospy.loginfo("Driving to waypoint (%f, %f)", waypoint[0], waypoint[1])
            	self.driveToWaypoint(waypoint)
            # Handle
            self.prev_coord_diff = coord_diff
            if rospy.is_shutdown() is True:
                break
        # Stopping our robot after the movement is over
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocityPublisher.publish(vel_msg)

        rospy.loginfo('Rotating to goal orientation (' + str(goalOrientation) + ')')
        
        # Finish off by rotating the robot to the final configuration
        if rospy.is_shutdown() is False:
            self.rotateToGoalOrientation(goalOrientation)
	run_time = self.get_run_time()
	print 'Previous path run time (s):' + str(run_time)
	self.total_run_time += run_time
	print 'Current total run time (s):' + str(self.total_run_time)
    
    def get_run_time(self):
        time_now = rospy.get_rostime().secs
        run_time = time_now - self.start_time
        return run_time

 
