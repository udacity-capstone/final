#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree

import math

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5
debug = True

class WaypointUpdater(object):
    def __init__(self):
        rospy.loginfo('Waypoint Updater Initializing')
        
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Lane, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)


        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.stopline_wp_index = None
        
        if debug:
            self.stopline_wp_index = 500

        #rospy.spin()


        rospy.loginfo('Waypoint Updater Initialized--Looping')
        self.loop()
        
    def loop(self):
        pubFreq_Hz = 50
        rate = rospy.Rate(pubFreq_Hz)
        
        while not rospy.is_shutdown():
            #rospy.logwarn('Waypoint Updater--pose: ' + self.pose)
            #rospy.logwarn('Waypoint Updater--pose: ' + self.base_waypoints)
            if self.pose and self.base_waypoints and self.waypoint_tree:
                #get the closest waypoint
                #closest_waypoint_index = self.get_closest_waypoint_index()
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_index(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_index = self.waypoint_tree.query([x,y],1)[1]
        
        #check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_index]
        prev_coord = self.waypoints_2d[closest_index - 1]
        
        #figure out if closest coordinate is infront or behind vehicle
        closest_coord_np = np.array(closest_coord)
        previous_coord_np = np.array(prev_coord)
        position_np = np.array([x,y])
        
        val = np.dot(closest_coord_np - previous_coord_np, position_np - closest_coord_np)
        
        #ensure closest index is for the closest waypoint ahead of the car
        if (val > 0):
            closest_index = (closest_index + 1) % len(self.waypoints_2d)
        
        #rospy.logwarn('Waypoint Updater--closest waypoint: ' + str(closest_index))
        
        return closest_index
    
    def publish_waypoints(self):
        lane = self.generate_lane()
        self.final_waypoints_pub.publish(lane)


    def generate_lane(self):
        lane = Lane()
        
        #generate waypoints
        closest_waypoint_index = self.get_closest_waypoint_index()
        farthest_waypoint_index = closest_waypoint_index + LOOKAHEAD_WPS
        base_waypoints = self.base_waypoints.waypoints[closest_waypoint_index: closest_waypoint_index + LOOKAHEAD_WPS]
        
        #decelerate waypoints if stopline is near
        if debug:
            rospy.logwarn('Waypoint -- closest waypoint ' + str(closest_waypoint_index))
            rospy.logwarn('Waypoint -- stopline_wp_index ' + str(self.stopline_wp_index) )
        if self.stopline_wp_index == None or self.stopline_wp_index == -1 or (self.stopline_wp_index >= farthest_waypoint_index):
            #do nothing if the stop waypoint is too far
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_waypoint_index)
        
        lane.header = self.base_waypoints.header
        return lane
    
    def decelerate_waypoints(self, waypoints, closest_index):
        
        
        new_waypoints = []
        
        for i, waypoint in enumerate(waypoints):
            new_waypoint = Waypoint()
            new_waypoint.pose = waypoint.pose
            
            #stop_index is a bit further back from the stop line
            stop_index = max(self.stopline_wp_index - closest_index -2, 0)
            dist = self.distance(waypoints, i, stop_index)
            vel = math.sqrt(2*MAX_DECEL*dist)
            if vel <1:
                vel = 0
            new_waypoint.twist.twist.linear.x = min(vel, waypoint.twist.twist.linear.x)
            
            new_waypoints.append(new_waypoint)
        if debug:    
            rospy.logwarn('Waypoint -- decelerating, velocity: ' + str(new_waypoints[0].twist.twist.linear.x ))
        return new_waypoints
            
    def pose_cb(self, msg):
        # TODO: Implement
        #rospy.logwarn('Waypoint -- loading pose') 
        self.pose = msg
        pass

    def waypoints_cb(self, waypoints):
        rospy.logwarn('Waypoint -- loading base waypoints')                  
        # TODO: Implement
        self.base_waypoints = waypoints
        rospy.logwarn(str(self.waypoints_2d))
        if not self.waypoints_2d:
            self.waypoints_2d = []
            for waypoint in waypoints.waypoints:
                self.waypoints_2d.append([waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] )
            self.waypoint_tree = KDTree(self.waypoints_2d)
            rospy.logwarn('Waypoint -- wauypoint_tree' + str(self.waypoint_tree))
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_index = msg
        pass

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
