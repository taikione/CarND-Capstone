#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Quaternion
from styx_msgs.msg import Lane, Waypoint

import numpy as np
import math
import tf

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

LOOKAHEAD_WPS = 40 # Number of waypoints we will publish. You can change this number
# LOOKAHEAD_WPS_DIST = 40.0
LOOKAHEAD_WPS_DIST = 60.0
# LOOKAHEAD_WPS_DIST = 100.0
RAD2DGREES = 57.32
DEG = 3


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # rospy.Subscriber('/obstacle_waypoints', , )
        # rospy.Subscriber('/traffic_waypoints', , )

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
        self.cte_pub = rospy.Publisher('cross_track_error', Float64, queue_size=1)

        # TODO: Add other member variables you need below
        # self.velocity = None
        self.basewaypoints = None
        # self.basewaypoint_pose_xs = None
        # self.current_position = PoseStamped()
        self.current_position = None
        self.yaw = None
        # self.current_head_direction = None

        # rospy.spin()

        self.loop()

    def loop(self):

        rate = rospy.Rate(50)

        while self.basewaypoints is None or self.current_position is None:

            rate.sleep()

        while not rospy.is_shutdown():

            self.yaw = self.get_current_car_direction()
            closest_wp_index = self.get_closest_waypoint_index()

            smooth_waypoints  = self.get_smooth_waypoints_coordinates(closest_wp_index, LOOKAHEAD_WPS_DIST)
            final_waypoints = self.get_waypoints(smooth_waypoints)

            self.final_waypoints_pub.publish(final_waypoints)
            cte = self.get_cte(smooth_waypoints)
            self.cte_pub.publish(cte)
            rospy.loginfo("yaw:{}, cte:{}, s_wp_len:{}".format(self.yaw*180/3.14, cte, len(smooth_waypoints)))

            rate.sleep()

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_position = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.basewaypoints = waypoints.waypoints

    def get_waypoints(self, smooth_waypoints):

        final_waypoints = Lane()
        final_waypoints.header.frame_id = '/world'
        final_waypoints.header.stamp = rospy.Time.now()

        for wp in smooth_waypoints:
             final_waypoint = wp
             final_waypoint.twist.twist.linear.x = 8.8 # 20kmph
             final_waypoints.waypoints.append(final_waypoint)

        return final_waypoints

    def get_polynomial(self, waypoints):

        wp_xs = [wp.pose.pose.position.x for wp in waypoints]
        wp_ys = [wp.pose.pose.position.y for wp in waypoints]

        try:
            poly = np.polyfit(wp_xs, wp_ys, DEG)

        except np.RankWarning:
            rospy.loginfo("RankWarning")

        return poly

    def get_smooth_waypoints_coordinates(self, start_index, wps_distance):

        # backward_wp = self.decimating_waypoints(start_index-1, 1.0, wps_distance, False)
        forward_wp = self.decimating_waypoints(start_index, 1.0, wps_distance, True)

        # poly = self.get_polynomial(backward_wp + forward_wp)
        poly = self.get_polynomial(forward_wp)

        wp_xs = [wp.pose.pose.position.x for wp in forward_wp]
        new_wp_ys = [np.polyval(poly, x) for x in wp_xs]
        for i in range(len(forward_wp)):
            # forward_wp[i].pose.pose.position.x = wp_xs[i]
            forward_wp[i].pose.pose.position.y = new_wp_ys[i]

        return forward_wp

    def decimating_waypoints(self, start_index, limit, target_distance, forward=True):

        waypoints = []
        index = start_index
        base_wp_index = 0
        distance = 0


        while distance <= target_distance and len(waypoints) < 40:

            if len(waypoints) == 0:
                waypoints.append(self.basewaypoints[index])
                base_wp_index = index

                translated_base_x, _ = self.get_translated_waypoint(waypoints[-1])
                distance += translated_base_x
                # rospy.loginfo("base_idx:{}, wps_dist:{}".format(base_wp_index, distance))
                rospy.loginfo("base_idx:{}, base_x:{}, base_y:{}, wps_dist:{}".format(
                    base_wp_index, self.basewaypoints[base_wp_index].pose.pose.position.x,
                    self.basewaypoints[base_wp_index].pose.pose.position.y, distance))

            else:

                base = self.basewaypoints[base_wp_index]
                target = self.basewaypoints[index]

                # rotate wps
                # translated_base_x, _ = self.get_translated_waypoint(base)
                # translated_target_x, _ = self.get_translated_waypoint(target)

                # convert frenet
                translated_base_x, _ = self.get_frenet_coordinate(base.pose, self.yaw, self.basewaypoints)
                translated_target_x, _ = self.get_frenet_coordinate(target.pose, self.yaw, self.basewaypoints)

                wps_distance = abs(translated_target_x - translated_base_x)


                if wps_distance > limit:
                    waypoints.append(target)

                    if base_wp_index > 4130: # for debug
                    # rospy.loginfo("base_idx:{}, target_idx:{}, wps_dist:{}".format(base_wp_index, index, wps_distance))
                        base_x = base.pose.pose.position.x
                        base_y = base.pose.pose.position.y
                        target_x = target.pose.pose.position.x
                        target_y = target.pose.pose.position.y

                        rospy.loginfo("base_idx:{}, base_x:{}, base_y:{}, target_idx:{}, target_x:{}, target_y:{}, tr_dist:{}, orgn_dist:{}".format(
                        base_wp_index, base_x, base_y, index, target_x, target_y, wps_distance, base_x - target_x))

                    base_wp_index = index

                    distance += wps_distance

            if forward:
                index += 1

            else:
                index -= 1

            index %= len(self.basewaypoints)

        return waypoints

    def get_closest_waypoint_index(self):

        distances = [self.get_distance_to_waypoints(waypoint.pose, self.current_position) for waypoint in self.basewaypoints]

        closest_waypoint_index = np.argmin(distances)

        forward_closest_waypoint_index = self.get_forward_waypoint_index(closest_waypoint_index)

        rospy.loginfo("current pose; x:{}, y:{}, z:{}".format(self.current_position.pose.position.x,
                                                              self.current_position.pose.position.y,
                                                              self.current_position.pose.position.z))

        rospy.loginfo("close pose; x:{}, y:{}, z:{}".format(self.basewaypoints[forward_closest_waypoint_index].pose.pose.position.x,
                                                             self.basewaypoints[forward_closest_waypoint_index].pose.pose.position.y,
                                                             self.basewaypoints[forward_closest_waypoint_index].pose.pose.position.z))

        rospy.loginfo("forward closest_waypoint index:{}\n".format(forward_closest_waypoint_index))

        return forward_closest_waypoint_index

    # def get_forward_waypoint_index(self, car_closest_wp_index, forward_distance):
    def get_forward_waypoint_index(self, car_closest_wp_index):

        index = car_closest_wp_index

        # view_angle = 60
        view_angle = 90

        while self.is_forward(index, view_angle) is False:

            index += 1
            index %= len(self.basewaypoints)

        return index

    def is_forward(self, wp_index, angle):

        is_forward = False
        current_x = self.current_position.pose.position.x
        current_y = self.current_position.pose.position.y

        ptsx = self.basewaypoints[wp_index].pose.pose.position.x - current_x
        ptsy = self.basewaypoints[wp_index].pose.pose.position.y - current_y

        translated_ptsx = ptsx*math.cos(-self.yaw) - ptsy*math.sin(-self.yaw)
        translated_ptsy = ptsx*math.sin(-self.yaw) + ptsy*math.cos(-self.yaw)

        target_wp_dir = math.ceil(math.atan2(translated_ptsy, translated_ptsx)*RAD2DGREES)

        # check target_wp_dir in 0 to 90 and -90 to 0
        if target_wp_dir in range(0, angle) or target_wp_dir in range(-angle, 0):
            is_forward = True

        rospy.loginfo("target index:{}, wp direction:{}, is:{}".format(wp_index, target_wp_dir, is_forward))

        return is_forward

    def get_cte(self, waypoints):

        translated_ptsx, translated_ptsy = self.get_translated_corrdinates(waypoints)

        polynomial = np.polyfit(translated_ptsx, translated_ptsy, DEG)
        cte = np.polyval(polynomial, 0)

        return cte


    def get_translated_corrdinates(self, waypoints):

        current_x = self.current_position.pose.position.x
        current_y = self.current_position.pose.position.y

        translated_ptsx = []
        translated_ptsy = []

        for wp in waypoints:

            ptsx = wp.pose.pose.position.x - current_x
            ptsy = wp.pose.pose.position.y - current_y

            translated_ptsx.append(ptsx*math.cos(-self.yaw) - ptsy*math.sin(-self.yaw))
            translated_ptsy.append(ptsx*math.sin(-self.yaw) + ptsy*math.cos(-self.yaw))

        return translated_ptsx, translated_ptsy

    def get_translated_waypoint(self, waypoint):

        current_x = self.current_position.pose.position.x
        current_y = self.current_position.pose.position.y

        ptsx = waypoint.pose.pose.position.x - current_x
        ptsy = waypoint.pose.pose.position.y - current_y

        translated_ptsx = ptsx*math.cos(-self.yaw) - ptsy*math.sin(-self.yaw)
        translated_ptsy = ptsx*math.sin(-self.yaw) + ptsy*math.cos(-self.yaw)

        return translated_ptsx, translated_ptsy

    def get_current_car_direction(self):
        quaternion = (self.current_position.pose.orientation.x,
                      self.current_position.pose.orientation.y,
                      self.current_position.pose.orientation.z,
                      self.current_position.pose.orientation.w)

        euler = tf.transformations.euler_from_quaternion(quaternion)

        return euler[2]

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def get_frenet_coordinate(self, target_waypoint, theta, base_waypoints):

        next_wp_index = self.get_next_wp_index(target_waypoint, theta, base_waypoints)
        prev_wp_index = next_wp_index - 1

        if next_wp_index == 0:
            prev_wp_index = len(base_waypoints) - 1

        prev_wp_x = base_waypoints[prev_wp_index].pose.pose.position.x
        prev_wp_y = base_waypoints[prev_wp_index].pose.pose.position.y

        nx = base_waypoints[next_wp_index].pose.pose.position.x - prev_wp_x
        ny = base_waypoints[next_wp_index].pose.pose.position.y - prev_wp_y
        xx = target_waypoint.pose.position.x - prev_wp_x
        xy = target_waypoint.pose.position.y - prev_wp_y

        # find the projection of x onto n
        proj_norm = (xx*nx + xy*ny) / (nx*nx + ny*ny)
        proj_x = proj_norm*nx
        proj_y = proj_norm*ny

        frenet_d = get_distance(xx, xy, proj_x, proj_y)

        center_x = 1000 - prev_wp_x
        center_y = 2000 - prev_wp_y
        center_to_pos = get_distance(center_x, center_y, xx, xy)
        center_to_ref = get_distance(center_x, center_y, proj_x, proj_y)

        if center_to_pos <= center_to_ref:
            frenet_d *= -1

        frenet_s = 0
        index = 0

        while index < prev_wp_index:

            wp_x = base_waypoints[index].pose.pose.position.x
            wp_y = base_waypoints[index].pose.pose.position.y

            next_wp_x = base_waypoints[index+1].pose.pose.position.x
            next_wp_y = base_waypoints[index+1].pose.pose.position.y

            frenet_s += get_distance(wp_x, wp_y, next_wp_x, next_wp_y)
            index += 1

        frenet_s += get_distance(0, 0, proj_x, proj_y)

        return frenet_s, frenet_d


    def get_next_wp_index(self, current_waypoint, theta, base_waypoints):

        distances = [self.get_distance_to_waypoints(current_waypoint, base_waypoint.pose) for base_waypoint in base_waypoints]
        closest_wp_index = np.argmin(distances)

        closest_wp = base_waypoints[closest_wp_index]

        diff_x = current_waypoint.pose.position.x - closest_wp.pose.pose.position.x
        diff_y = current_waypoint.pose.position.y - closest_wp.pose.pose.position.y

        heading = math.atan2(diff_y, diff_x)
        angle = abs(theta - heading)

        if angle > (math.pi/4):
            closest_wp_index += 1

        return closest_wp_index


    def get_distance_to_waypoints(self, waypoint1, waypoint2):

        wp1_coordinate = waypoint1.pose.position
        wp2_coordinate = waypoint2.pose.position

        diff_x = wp1_coordinate.x - wp2_coordinate.x
        diff_y = wp1_coordinate.y - wp2_coordinate.y

        return math.sqrt(diff_x**2 + diff_y**2)

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


def get_distance(x1, y1, x2, y2):

    diff_x = x2 - x1
    diff_y = y2 - y1

    return math.sqrt(diff_x**2 + diff_y**2)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
