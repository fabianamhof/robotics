import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('test', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    x = 0
    while not rospy.is_shutdown():
        rospy.loginfo(str(x))
        pub.publish(str(x))
        x = x + 0.1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass