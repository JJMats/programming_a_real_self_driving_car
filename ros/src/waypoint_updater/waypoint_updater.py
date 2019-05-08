#!/usr/bin/env python

import math
import numpy as np
import rospy
import yaml
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
MAX_ACCEL = 1.0 #l m/s - Maximum acceleration rate to keep jerk below 10m/s^3
MAX_DECEL = 4.0 # m/s - Maximum deceleration rate to keep jerk below 10m/s^3
TARGET_DECEL_RATE = 1.0 # m/s

class WaypointUpdater(object):
    def __init__(self):        
        rospy.init_node('waypoint_updater')  

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        # TODO: Add other member variables you need below
        self.base_lane = None
        self.pose = None
        self.stopline_wp_idx = -1
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.current_vel = 0.0
        self.light_state = -1
        self.is_simulation = not self.config["is_site"]

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)    

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)     
        rospy.Subscriber('/traffic_light_state', Int32, self.traffic_light_state_cb)
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
        
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx > farthest_idx) or \
            (self.light_state > 0 and self.light_state < 4):
            # If current velocity < waypoint velocity - threshold (maybe 80%?), accelerate to it
            #rospy.logwarn("Current velocity: {0}, target velocity: {1}, Light State: {2}".format(self.current_vel, base_waypoints[0].twist.twist.linear.x, self.light_state))
            if not self.is_simulation and self.current_vel < base_waypoints[0].twist.twist.linear.x * 0.8:
                #rospy.logwarn("Go to accelerate_waypoints")
                lane.waypoints = self.accelerate_waypoints(base_waypoints)
            else:
                lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
            
        return lane
    
    def accelerate_waypoints(self, waypoints):
        temp = []
        last_vel = self.current_vel
        last_time_to_next_wp = 0.0

        #rospy.logwarn("Waypoint length: {0}".format(len(waypoints)))  

        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            time_to_next_wp = 0.0
            dist = 0.0
            
            # To calculate accel to be added, it is necessary to determine the time it will
            #  take at the current velocity to reach the next waypoint
            
            if i == len(waypoints) - 1:
                new_vel = last_vel + last_time_to_next_wp * MAX_ACCEL
            else:
                dist = self.distance(waypoints, i, i+1)
            
                if last_vel < 1.0:
                    new_vel = 1.0
                else:
                    time_to_next_wp = dist / last_vel
                    new_vel = last_vel + (MAX_ACCEL * time_to_next_wp)
            
            #rospy.logwarn("Last velocity: {0}, New velocity: {1}, TTNWP: {2}, Dist: {3}".format(last_vel, new_vel, time_to_next_wp, dist))
            new_vel = min(new_vel, wp.twist.twist.linear.x)
            last_vel = new_vel
            last_time_to_next_wp = time_to_next_wp

            p.twist.twist.linear.x = new_vel
            temp.append(p)

        return temp


    
    def decelerate_waypoints(self, waypoints, closest_idx):
        temp = []
        stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0) # Two waypoints back from stop line so front of car stops before stop line
        last_braking_velocity = self.current_vel
        new_dec_rate = TARGET_DECEL_RATE

        #rospy.logwarn("")
        #rospy.logwarn("*** Calculating decel waypoints: Stopline idx: {0}, Closest idx:{1}, Stop idx: {2}".format(self.stopline_wp_idx, closest_idx, stop_idx))
        for i, wp in enumerate(waypoints):            
            p = Waypoint()
            p.pose = wp.pose
            braking_vel = 0.0         
            dist = self.distance(waypoints, i, stop_idx)

            # ** Maybe change this dist value to dist > 1.0 to prevent rolling forward?
            if dist > 3.0 and last_braking_velocity < 1.0:
                # Allow vehicle to accelerate from a dead stop if it is away from the stop line
                # This is needed to get the car to start moving if the traffic light is red

                # Calculate time to decelerate for the current distance at the deceleration rate
                tts = math.sqrt(dist * 2.0 / new_dec_rate)

                # Calculate braking velocity (a* t)
                braking_vel = new_dec_rate * tts

                # Clamp to maximum velocity parameter
                braking_vel = min(braking_vel, wp.twist.twist.linear.x)

            elif dist > 0.0 and not last_braking_velocity == 0.0:
                tts = 0.0

                if i == 0:
                    # Determine if the vehicle needs to decelerate at a rate faster than
                    #  the TARGET_DECEL_RATE, which is typically caused by:
                    #  - Missed classifications that cause a light state change
                    #  - Late detection of a stop light
                    #  - YELLOW to RED light transitions

                    # Calculate time to decelerate at the last known velocity
                    tts = dist * 2.0 / last_braking_velocity

                    # Calculate the deceleration rate needed to stop the vehicle from the
                    #  last known velocity and time to stop
                    req_dec_rate = last_braking_velocity / tts

                    # Grab the higher decel rate - This will prevent pre-mature deceleration
                    req_dec_rate = max(req_dec_rate, TARGET_DECEL_RATE)

                    # Clamp the decel rate to the MAX_ACCEL parameter
                    new_dec_rate = min(req_dec_rate, MAX_DECEL)

                else:
                    # Continue to decelerate at the last calculated decel rate

                    # Calculate time to decelerate for the current distance at the
                    #  deceleration rate
                    tts = math.sqrt(dist * 2.0 / new_dec_rate)

                # Calculate braking velocity (a * t)
                braking_vel = new_dec_rate * tts

                # Clamp to maximum velocity parameter
                braking_vel = min(braking_vel, wp.twist.twist.linear.x)

                # Hold car at a stop if the velocity is below 1.0 m/s
                if braking_vel < 0.5:
                    braking_vel = 0.0

                # Set last braking velocity to the new velocity so that this IF block can
                #  be bypassed on the rest of the waypoints
                last_braking_velocity = braking_vel

                # rospy.logwarn("Curr Vel: {0}, Braking Vel: {1}, Dist: {2}, New Decel Rate: {3}, \
                #                 TTS: {4}" \
                #     .format(last_braking_velocity, braking_vel, dist, new_dec_rate, \
                #         tts))
            
            # Original function
            #vel = math.sqrt(2 * MAX_DECEL * dist)
            #if vel < 0.5:
            #    vel = 0.0

            p.twist.twist.linear.x = braking_vel
            temp.append(p)

        return temp
            
        
    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg

    def velocity_cb(self, msg):
        self.current_vel = msg.twist.linear.x

        
    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_lane = waypoints        
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

            
    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data

    def traffic_light_state_cb(self, msg):
        self.light_state = msg.data

        
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
