#!/usr/bin/env python
"""
@brief - A node that publishes all of the topics needed for the bci experiments when there is no kinect

This means it must publish the camera to world pose that would normally be derived from the checkboard_detection package
and the pc_filtered topic

FIXME - Consider adding fake visual data from camera too and maybe unfiltered point cloud option
"""

import roslib

roslib.load_manifest('pc_filter')
roslib.load_manifest('rosbag')
roslib.load_manifest('sensor_msgs')
import sensor_msgs
import rospy
import rosbag
import tf
import pdb
import time
import calendar

class CloudDataPublisher(object):
    def __init__(self, bag):
        def get_tf(parent_frame, child_frame, bag):
            r = bag.read_messages('/tf')
            for recorded_message in r:
                m = recorded_message[1]
                if m.transforms[0].child_frame_id == child_frame and m.transforms[0].header.frame_id == parent_frame:
                    return m
                    
        #Get TFs
        camera_to_world = get_tf('/camera_rgb_optical_frame','/world', bag)        
        rgb_to_rgb_optical = get_tf('/camera_rgb_frame','/camera_rgb_optical_frame',bag)
        depth_to_depth_optical = get_tf('/camera_depth_frame','/camera_depth_optical_frame',bag)
        camera_link_to_rgb = get_tf('/camera_link','/camera_rgb_frame',bag)
        camera_link_to_depth = get_tf('/camera_link','/camera_depth_frame',bag)


        r = bag.read_messages('/filtered_pc')
        self.point_cloud = r.next()[1]
        self.point_cloud.header.stamp.secs = calendar.timegm(time.gmtime())
        self.tf_msgs = [camera_to_world, rgb_to_rgb_optical, depth_to_depth_optical,
                        camera_link_to_rgb, camera_link_to_depth]
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.point_cloud_pub = rospy.Publisher(rospy.get_param("cloud_topic","/pc_filter"),sensor_msgs.msg.PointCloud2)

    def publish(self):
        for msg in self.tf_msgs:
            translation = msg.transforms[0].transform.translation
            t = [translation.x, translation.y, translation.z]
            rotation = msg.transforms[0].transform.rotation
            r = [rotation.x, rotation.y, rotation.z, rotation.w]

            self.tf_broadcaster.sendTransform(t, r, rospy.Time.now(), msg.transforms[0].child_frame_id,msg.transforms[0].header.frame_id)

            
        self.point_cloud_pub.publish(self.point_cloud)

if __name__ == '__main__':
    rospy.init_node('fake_kinect_data')
    loop = rospy.Rate(30)
    bag = rosbag.Bag(roslib.packages.get_pkg_dir('bci_experiment_launch') +'/scripts/simulate_kinect_data.bag','r')
    cloud_pub = CloudDataPublisher(bag)
    while not rospy.is_shutdown():
        cloud_pub.publish()
        loop.sleep()
        
    



            
