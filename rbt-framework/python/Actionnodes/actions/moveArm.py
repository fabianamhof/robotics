import py_trees
import rospy
import action
import numpy as np
from rospy.rostime import get_time

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from std_msgs.msg import Int8


def get_quaternion_from_euler(euler):
    """
    Convert an Euler angle to a quaternion.

    Input
      :param euler.x: The euler.x (rotation around x-axis) angle in radians.
      :param euler.y: The euler.y (rotation around y-axis) angle in radians.
      :param euler.z: The euler.z (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(euler.x / 2) * np.cos(euler.y / 2) * np.cos(euler.z / 2) - np.cos(euler.x / 2) * np.sin(euler.y / 2) * np.sin(euler.z / 2)
    qy = np.cos(euler.x / 2) * np.sin(euler.y / 2) * np.cos(euler.z / 2) + np.sin(euler.x / 2) * np.cos(euler.y / 2) * np.sin(euler.z / 2)
    qz = np.cos(euler.x / 2) * np.cos(euler.y / 2) * np.sin(euler.z / 2) - np.sin(euler.x / 2) * np.sin(euler.y / 2) * np.cos(euler.z / 2)
    qw = np.cos(euler.x / 2) * np.cos(euler.y / 2) * np.cos(euler.z / 2) + np.sin(euler.x / 2) * np.sin(euler.y / 2) * np.sin(euler.z / 2)

    return Quaternion(qx, qy, qz, qw)


class Action(action.Action):
    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is
        to only include the initialisation relevant for being able
        to insert this behaviour in a tree for offline rendering to
        dot graphs.

        Other one-time initialisation requirements should be met via
        the setup() method.
        """
        super(Action, self).__init__(name)
        self.pub = None
        self.blackboard = None
        self.name = name
        self.executed = False
        self.published = False
        self.publishedTime = get_time()

    def setup(self):
        """
        When is this called?
          This function should be either manually called by your program
          to setup this behaviour alone, or more commonly, via
          :meth:`~py_trees.behaviour.Behaviour.setup_with_descendants`
          or :meth:`~py_trees.trees.BehaviourTree.setup`, both of which
          will iterate over this behaviour, it's children (it's children's
          children ...) calling :meth:`~py_trees.behaviour.Behaviour.setup`
          on each in turn.

          If you have vital initialisation necessary to the success
          execution of your behaviour, put a guard in your
          :meth:`~py_trees.behaviour.Behaviour.initialise` method
          to protect against entry without having been setup.

        What to do here?
          Delayed one-time initialisation that would otherwise interfere
          with offline rendering of this behaviour in a tree to dot graph
          or validation of the behaviour's configuration.

          Good examples include:

          - Hardware or driver initialisation
          - Middleware initialisation (e.g. ROS pubs/subs/services)
          - A parallel checking for a valid policy configuration after
            children have been added or removed
        """
        self.blackboard = py_trees.blackboard.Client(name="moveArm")
        self.blackboard.register_key(key="targetPosition", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="targetPosition", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="targetOri", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="robot_state", access=py_trees.common.Access.WRITE)
        self.pub = rospy.Publisher('target_pos', Pose, queue_size=1000)

        return

    def initialise(self):
        """
        When is this called?
          The first time your behaviour is ticked and anytime the
          status is not RUNNING thereafter.

        What to do here?
          Any initialisation you need before putting your behaviour
          to work.
        """
        self.logger.debug("  %s [Foo::initialise()]" % self.name)

    def update(self):
        """
        When is this called?
          Every time your behaviour is ticked.

        What to do here?
          - Triggering, checking, monitoring. Anything...but do not block!
          - Set a feedback message
          - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        """
        if self.published and self.publishedTime + 5 < get_time():
            self.published = False
            return py_trees.common.Status.FAILURE

        if self.blackboard.robot_state == Int8(1) and self.published:
            self.published = False
            return py_trees.common.Status.SUCCESS
        if not self.published:
            if self.name == "moveArmUp":
                print("moving Arm up")
                self.blackboard.targetPosition = Point(self.blackboard.targetPosition.x + 0.1, self.blackboard.targetPosition.y, self.blackboard.targetPosition.z)
            self.blackboard.set("robot_state", Int8(0))
            target_pos = Pose()
            target_pos.position = self.blackboard.targetPosition
            target_pos.orientation = get_quaternion_from_euler(self.blackboard.targetOri)
            self.pub.publish(target_pos)
            self.published = True
            self.publishedTime = get_time()
            print("Published Arm position")

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """
        When is this called?
           Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
