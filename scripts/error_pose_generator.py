#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point, Quaternion

def myhook():
  print "shutdown time!"
def main():
    #pub = rospy.Publisher('chatter', String, queue_size=10)

    rospy.init_node('talker', anonymous=True)
    error_direction = rospy.get_param('~direction')
    error_value = rospy.get_param('~value')
    type_list = [ "position", "orientation" ]
    dir_list = ["x", "y", "z", "o"]

    if not error_direction in dir_list:
        rospy.logerr('the direction is selected form "x", "y", "z", "o":%s ', error_direction)
        exit()
    rospy.loginfo('error direction is :%s ,the value is :%s', error_direction, error_value)
    r = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        if error_direction == "w":
            pass
            # error_value => degree to 
        else:
            if error_direction == "x":
                error_pose.x += error_value
            if error_direction == "y":
                error_pose.y += error_value
            if error_direction == "z":
                error_pose.z += error_value
            pub_pose.publish(error_pose)
        rospy.loginfo(error_pose)
        r.sleep()

if __name__ == '__main__':
    try:
        NODE_NAME = 'error_pose_generator'
        pub_pose = rospy.Publisher("/AR/confution_pose/position", Point, queue_size=10)
        pub_ori = rospy.Publisher("/AR/confution_pose/orientation", Quaternion, queue_size=10)
        error_pose = Point()
        main()
    except rospy.ROSInterruptException: 
        pass