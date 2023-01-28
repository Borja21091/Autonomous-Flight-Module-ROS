#!/usr/bin/env python

import rospy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from flight_control import Drone

class FixedTFBroadcaster(object):
    def __init__(self):
        self.pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=1)
        self.drone = Drone()
        self.pos = self.drone.get_position()
        
    def start(self):

        while not rospy.is_shutdown():

            t = TransformStamped()
            t.header.frame_id = "vicon/world"
            t.header.stamp = rospy.Time.now()
            t.child_frame_id = "trajectory0"
            t.transform.translation.x = 1 #self.pos[0,0]
            t.transform.translation.y = 1 #self.pos[1,0]
            t.transform.translation.z = 1 #self.pos[2,0]

            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0

            tfm = TFMessage([t])
            self.pub_tf.publish(tfm)
            
            # Run this loop at about 10Hz
            rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        # Init ROS Node
        rospy.init_node('trajectory0_frame', anonymous = True)
        tfb = FixedTFBroadcaster()
        tfb.start()
        # rospy.spin()

    except rospy.ROSInterruptException:
        pass