#!/usr/bin/env python3
import imp
import re
import rospy
import rosnode
from std_srvs.srv import Trigger, TriggerResponse
import roslaunch
import rospkg
import os
from robot_localization.srv import SetPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from phoenix_msgs.srv import (
        StartLocalization, StartLocalizationRequest, StartLocalizationResponse
)
import tf2_ros
import time

class SlamLaunchManager:
    def __init__(self) -> None:
        rospy.init_node('slam_launch_manager')
        rospy.Service('activate_slam_toolbox', Trigger, self.activate_slam_cb)
        rospy.Service('deactivate_slam_toolbox', Trigger, self.deactivate_slam_cb)
        rospy.Service('activate_slam_toolbox_localization', StartLocalization, self.activate_localization_cb)
        rospy.Service('deactivate_slam_toolbox_localization', Trigger, self.deactivate_localization_cb)

        rospy.loginfo('Waiting for /rail_navigation/start_localization')
        rospy.wait_for_service('/rail_navigation/start_localization')
        rospy.loginfo('Received /rail_navigation/start_localization')

        self.initialize_localization_srv = rospy.ServiceProxy(
            '/rail_navigation/start_localization', StartLocalization
        )

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.reset_ekf_srv = rospy.ServiceProxy('set_pose', SetPose)
        rospack = rospkg.RosPack()
        ros_pkg_dir = rospack.get_path('phoenix1_bringup')
        print(ros_pkg_dir)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        print(os.path.join(ros_pkg_dir, "launch/slam_launch.launch"))
        r = rospy.Rate(10)
        self.slam_launch = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(ros_pkg_dir, "launch/slam_launch.launch")])
        self.localization_launch = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(ros_pkg_dir, "launch/slam_premapped_launch.launch")])

        self.slam_is_launched = False
        self.request_slam_deactivate = False
        self.request_slam_launch = False

        self.localization_is_launched = False
        self.request_localization_deactivate = False
        self.request_localization_launch = False

        self.start_rail_number = 1

        while not rospy.is_shutdown():
            if not self.slam_is_launched and self.request_slam_launch:
                rospy.loginfo('Activating SlAM toolbox')
                self.slam_launch.start()
                self.slam_is_launched = True
                self.request_slam_launch = False

            if self.slam_is_launched and self.request_slam_deactivate:
                self.slam_launch.shutdown()
                self.slam_launch = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(ros_pkg_dir, "launch/slam_launch.launch")])
                self.slam_is_launched = False
                self.request_slam_deactivate = False

            if not self.localization_is_launched and self.request_localization_launch:
                rospy.loginfo('Activating SlAM toolbox localization')
                self.localization_launch.start()

                # wait for localization to come up
                rospy.sleep(1.0)
                self.initialize_pose()

                self.request_localization_launch = False
                self.localization_is_launched = True

            if self.localization_is_launched and self.request_localization_deactivate:
                self.localization_launch.shutdown()
                self.localization_launch = roslaunch.parent.ROSLaunchParent(uuid, [os.path.join(ros_pkg_dir, "launch/slam_premapped_launch.launch")])
                self.request_localization_deactivate = False
                self.localization_is_launched = False
            r.sleep()

    @staticmethod
    def wait_for_event(event, timeout=5, check_interval=0.15):
        start_time = rospy.Time.now()
        success = False
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            if event():
                success = True
                break
            rospy.sleep(check_interval)

        return success

    def initialize_pose(self):
        try:
            req = StartLocalizationRequest()
            req.rail_number = self.start_rail_number
            result = self.initialize_localization_srv(req)
        except rospy.ServiceException as e:
            rospy.logerr("Initialize pose Service call failed: %s" % e)

    def activate_localization_cb(self, req):
        self.start_rail_number = req.rail_number
        res = StartLocalizationResponse()
        if self.localization_is_launched:
            res.success = False
            res.message = "Localization is already Launched!"
        else:
            pose_msg = PoseWithCovarianceStamped()
            try:
                self.reset_ekf_srv(pose_msg)
            except rospy.ServiceException:
                res.success = False
                res.message = "Cannot reset EKF node!"
                return res

            self.request_localization_launch = True

            if self.wait_for_event(
                lambda : (not self.request_localization_launch) and (self.localization_is_launched)
            ):
                res.success = True
                res.message = "Localization activated"
            else:
                res.success = False
                res.message = "Localization timeout"

        return res

    def deactivate_localization_cb(self, req):
        res = TriggerResponse()
        if self.localization_is_launched:
            self.request_localization_deactivate = True

            if self.wait_for_event(
                lambda : (not self.request_localization_deactivate) and (not self.localization_is_launched)
            ):
                res.success = True
                res.message = "Localization deactivated"
            else:
                res.success = False
                res.message = "Localization timeout"
        else:
            res.success = False
            res.message = "Localization already inactive"

        return res

    def activate_slam_cb(self, req):
        res = TriggerResponse()
        if self.slam_is_launched:
           res.success = False
           res.message = "SLAM is already Launched!"
        else:
            pose_msg = PoseWithCovarianceStamped()
            try:
                self.reset_ekf_srv(pose_msg)
            except rospy.ServiceException:
                res.success = False
                res.message = "Cannot reset EKF node!"
                return res
            self.request_slam_launch = True

            if not self.wait_for_event(
                lambda : (not self.request_slam_launch) and (self.slam_is_launched)
            ):
                res.success = False
                res.message = "SLAM timeout"
                return res

            time.sleep(1)

            if self.wait_for_event(
                lambda : self.tfBuffer.can_transform('base_link', 'map', rospy.Time(), rospy.Duration(0.5))
            ):
                res.success = True
                res.message = "SLAM node is launched Successfuly. :)"
            else:
                res.success = False
                res.message = "Failed to activate EKF! TF conenection issue between map and baselink"

        return res

    def deactivate_slam_cb(self, req):
        res = TriggerResponse()
        if self.slam_is_launched:
            self.request_slam_deactivate = True

            if self.wait_for_event(
                lambda : (not self.request_slam_deactivate) and (not self.slam_is_launched)
            ):
                res.success = True
                res.message = "SLAM toolbox is deactivated. :("
            else:
                res.success = False
                res.message = "SLAM toolbox timeout"
        else:
            res.success = False
            res.message = "SLAM is already deactive!"
        return res



if __name__ == '__main__':
    p = SlamLaunchManager()
    rospy.spin()
