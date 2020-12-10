#!/usr/bin/env python
import rospy
import tf
import math
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point, Quaternion

def main():
    rospy.init_node(NODE_NAME, anonymous=True)
    rospy.loginfo("Publish to the confution_pose %s", rospy.Time.now())

    error_direction = rospy.get_param('~direction')
    error_value = rospy.get_param('~value')
    type_list = [ "position", "orientation" ]
    dir_list = ["x", "y", "z", "w"]

    if not error_direction in dir_list:
        rospy.logerr('the direction is selected form "x", "y", "z", "o":%s ', error_direction)
        exit()
    rospy.loginfo('error direction is :%s ,the value is :%s', error_direction, error_value)
    r = rospy.Rate(1) # 1hz
    #tf.quaternion( error_quat.x, error_quat.y, error_quat.z, error_quat.w)
    #tf.convert(error_quat , quat_tf)
    while not rospy.is_shutdown():
        if error_direction == "w":
            error_deg.data += error_value
            pub_deg.publish(error_deg)
            rospy.loginfo( error_deg)
            '''
            rad_value = error_value * math.pi / 180
            rad_quat = tf.transformations.quaternion_from_euler(0.0, 0.0, error_value)
            print(rad_quat)
            #error_quat.x
            #error_quat.y
            # error_value => degree to
            #pub_ori.publish(error_pose)
            error_quat *= rad_quat
            #q_new = quaternion_multiply(q_rot, q_orig)
            rospy.loginfo(error_quat)
            '''
        else:
            if error_direction == "x":
                error_pose.x += error_value
            if error_direction == "y":
                error_pose.y += error_value
            if error_direction == "z":
                error_pose.z += error_value
            pub_pose.publish(error_pose)
            #rospy.loginfo('error_generator add the pose: ', error_pose)
        r.sleep()

if __name__ == '__main__':
    try:
        NODE_NAME = 'error_pose_generator'
        pub_pose = rospy.Publisher("/RB/confution_pose/position", Point, queue_size=10)
        pub_ori = rospy.Publisher("/RB/confution_pose/orientation", Quaternion, queue_size=10)
        pub_deg = rospy.Publisher("/RB/confution_pose/degree", Float32, queue_size=10)
        error_pose = Point()
        error_quat = Quaternion()
        error_quat.w = 1.0
        error_deg = Float32()
        main()
    except rospy.ROSInterruptException: 
        pass