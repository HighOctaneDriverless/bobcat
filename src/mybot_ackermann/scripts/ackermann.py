#!/usr/bin/env python

import rospy
from std_msgs import msg
from std_msgs.msg import Float64
#from std_msgs.msg import String

# def callback(data):
#   rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

# def listener():
#   rospy.init_node('')
#   rospy.Subscriber("/mybot/left_rear_wheel_effort_controller/command",)

def talker():
    pub_left = rospy.Publisher("/mybot/left_rear_wheel_effort_controller/command", Float64, queue_size=10)
    pub_right = rospy.Publisher("/mybot/right_rear_wheel_effort_controller/command", Float64, queue_size=10)
    pub_steering_left = rospy.Publisher("/mybot/left_steering_hinge_position_controller/command", Float64, queue_size=10)
    pub_steering_right = rospy.Publisher("/mybot/right_steering_hinge_position_controller/command", Float64, queue_size=10)
    rospy.init_node('test_rear_wheel', anonymous=True)
    rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        go = 0.2
        steering = 0.0
        hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        rospy.loginfo(go)
        pub_left.publish(go)
        pub_right.publish(go)
        pub_steering_right.publish(steering)
        pub_steering_left.publish(steering)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
