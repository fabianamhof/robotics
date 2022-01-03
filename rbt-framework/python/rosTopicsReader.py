import rospy
import py_trees
import geometry_msgs.msg
import std_msgs.msg

class RTReader:
    def __init__(self, topic_list):
        rospy.init_node("rbtTopicsReader", anonymous=True, disable_signals=True)
        print("Rospy init node");
        self.blackboard = py_trees.blackboard.Client(name="rTR")
        self.topicList = topic_list
        self.SubList = []
        for topic in self.topicList:
            self.blackboard.register_key(key=topic["name"], access=py_trees.common.Access.WRITE)
            self.blackboard.set(topic["name"], eval(topic["init"]))
            self.SubList.append(rospy.Subscriber(topic["name"], eval(topic["type"]), self.callback, topic))



    def callback(self, value, topic):
        self.blackboard.set(topic["name"], value)

    def __del__(self):
        rospy.signal_shutdown("RTReader got destroyed")
