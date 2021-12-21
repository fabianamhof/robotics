import py_trees
import rospy
from rospy.rostime import get_time
import action


from std_msgs.msg import Bool
from std_msgs.msg import Int8


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
        self.publishedCommand = None

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
        self.blackboard = py_trees.blackboard.Client(name="controlGripper")
        self.blackboard.register_key(key="gripperPosition", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="gripperPosition", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="gripper_state", access=py_trees.common.Access.WRITE)
        self.pub = rospy.Publisher('gripper_pos', Bool, queue_size=1000)

        #rospy.init_node('rbt_node', anonymous=True)

        print("controlGripper; Setup")
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
        print("controlGripper; Update")

        if self.published:
            if (self.blackboard.gripper_state == Int8(1)) == (self.publishedCommand == Bool(True)):
                self.published = False
                return py_trees.common.Status.SUCCESS
            else:
                if self.publishedTime + 5 < get_time():
                    self.published = False
                    return py_trees.common.Status.FAILURE
                else:
                    return py_trees.common.Status.RUNNING
        if not self.published:
            if self.name == "openGripper":
                self.blackboard.set("gripperPosition", Bool(False))
            elif self.name == "closeGripper":
                self.blackboard.set("gripperPosition", Bool(True))
            p = self.blackboard.gripperPosition
            self.pub.publish(p)
            self.published = True
            self.publishedTime = get_time()
            self.publishedCommand = p
            print("Published Gripper position")

        return py_trees.common.Status.RUNNING



    def terminate(self, new_status):
        """
        When is this called?
           Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
