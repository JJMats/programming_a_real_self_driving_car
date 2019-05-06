#!/usr/bin/env python

import math
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from scipy.spatial import KDTree
from std_msgs.msg import Int32
from styx_msgs.msg import Lane, Waypoint

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 5.0 # m/s - Maximum deceleration rate to keep jerk below 10m/s^3
TARGET_DECEL_RATE = 2.5 # m/s

class WaypointUpdater(object):
    def __init__(self):        
        rospy.init_node('waypoint_updater')        

        # TODO: Add other member variables you need below
        self.base_lane = None
        self.pose = None
        self.stopline_wp_idx = -1
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.current_vel = 0.0

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)    

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)     
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        
        # **** Figure out what this datatype is. Do we need to create this message? ****
        #rospy.Subscriber('/obstacle_waypoint', ?, self.obstacle_cb)          
        
        self.loop()

        
    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                self.publish_waypoints()
            rate.sleep()

            
    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        
        # Each waypoint in the waypoint_tree is stored as [position, index]
        # The query() function will return the closest waypoint to [x, y], and
        #  the "1" value specifies to return only one item. We are then taking
        #  only the index ([1])
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        
        # Check if the closest waypoint is ahead of, or behind the vehicle
        # We are looking for the waypoint in front of the vehicle here
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]
        
        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])
        
        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)
        # Alternatively, you can take the orientation of the vehicle, and the 
        #  orientation of a vector from the previous waypoint to the current
        #  waypoint and compare them to determine if they are facing in the 
        #  same direction.
                
        if val > 0:
            # Waypoint is behind the vehicle, so increment index forward by one
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    
    def publish_waypoints(self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

        
    def generate_lane(self):
        lane = Lane()
        
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]
        
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx > farthest_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
            
        return lane
    
    
    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        stop_idx = max(self.stopline_wp_idx - closest_idx - 4, 0) # Four waypoints back from stop line so front of car stops before stop line
        
        #rospy.logwarn("")
        #rospy.logwarn("*** Calculating decel waypoints: Stopline idx: {0}, Closest idx:{1}, Stop idx: {2}".format(self.stopline_wp_idx, closest_idx, stop_idx))
        for i, wp in enumerate(waypoints):            
            p = Waypoint()
            p.pose = wp.pose
            braking_vel = 0.0         
            dist = self.distance(waypoints, i, stop_idx)
            
            if dist > 0.0:
                
            # Possibly change the following function to smooth the initial deceleration and final deceleration rates
            #rospy.logwarn("Dist: {0}, MAX_DECEL: {1}".format(dist,MAX_DECEL))

                # Calculate the time in seconds it takes to stop at the stop line
                #  at the Target Deceleration Rate
                
                #time_to_complete_stop = math.sqrt(dist * 2.0 / TARGET_DECEL_RATE) # Seconds
                time_to_complete_stop = 0.0

                # Calculate the distance required to stop at the Target Deceleration Rate
                stopping_dist_at_target_decel = dist / TARGET_DECEL_RATE

                if stopping_dist_at_target_decel > dist:
                    # Adjust Decel Rate up to Maximum to meet stopping distance
                    
                    # Calculate Required Deceleration Rate To Stop at Stop Line
                    required_decel_rate = self.current_vel / dist

                    # Clamp required decel rate to maximum deceleration rate
                    required_decel_rate = min(required_decel_rate, MAX_DECEL)
                    time_to_complete_stop = math.sqrt(dist * 2.0 / required_decel_rate)
                else:
                    time_to_complete_stop = math.sqrt(dist * 2.0 / TARGET_DECEL_RATE)
                
                braking_vel = 2.0 * dist / time_to_complete_stop
                
                if braking_vel < 0.05:
                    braking_vel = 0.0
            
            # Original function
            #vel = math.sqrt(2 * MAX_DECEL * dist)
            #if vel < 0.5:
            #    vel = 0.0

            #if braking_vel > 0.0:
            #    rospy.logwarn("WP idx: {0}, Time to stop: {1}, dist: {2}, braking_vel: {3}, linear_x: {4}".format(i,time_to_complete_stop,dist,braking_vel,wp.twist.twist.linear.x))
            
            # Take minimum velocity of braking_velocity and maximum velocity to prevent
            #   velocity changes over the maximum velocity at large dist values.
            
            p.twist.twist.linear.x = min(braking_vel, wp.twist.twist.linear.x)
            temp.append(p)
            
        return temp
            
        
    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg

    def velocity_cb(self, msg):
        self.current_vel = msg

        
    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_lane = waypoints        
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

            
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data

        
    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    
    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    
    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

        
    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
