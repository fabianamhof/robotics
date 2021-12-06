import rospy
import py_trees
from std_msgs.msg import String

class RTReader:
    def __init__(self, topic_list):
        rospy.init_node("rbtTopicsReader", anonymous=True, disable_signals=True)
        self.blackboard = py_trees.blackboard.Client(name="rTR")
        self.topicList = topic_list
        self.SubList = []
        for topic in self.topicList:
            self.blackboard.register_key(key=topic, access=py_trees.common.Access.WRITE)
            self.blackboard.set(topic, None)
            self.SubList.append(rospy.Subscriber(topic, String, self.callback, topic))



    def callback(self, value, topic):
        self.blackboard.set(topic, value.data)

    def __del__(self):
        rospy.signal_shutdown("RTReader got destroyed")