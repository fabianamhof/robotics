import time

import py_trees
import action
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3

basicPosition = Point(0.4, 0, 0.4)
basicOrienation = Vector3(0, np.pi / 2, np.pi)
placingPosition = Point(0.1, 0.2, 0.6)
placingOrientation = Vector3(0, np.pi / 2, np.pi)

#offset = 0, range = 90, returns angles between 315 and 45 degrees
def normalizeAngle(offset, range, angle):
    new_angle = ((angle - (offset-range/2))%range) + (offset-range/2)
    return new_angle

def normalizeAngles(angles):
    print("Normalizing " +  str(angles))
    x = normalizeAngle(basicOrienation.x, np.pi/2, angles.x)
    y = normalizeAngle(basicOrienation.y, np.pi/2, angles.y)
    z = normalizeAngle(basicOrienation.z, np.pi/2, angles.z)
    print("...to " +  str(Vector3(x,y,z)))
    return Vector3(x,y,z)


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
        self.blackboard = None
        self.name = name
        self.executed = False


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
        self.logger.debug("  %s [Foo::setup()]" % self.name)
        self.blackboard = py_trees.blackboard.Client(name="calcPos")
        self.blackboard.register_key(key="targetPosition", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="targetPosition", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="targetOri", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="cube_pos", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="cube_ori", access=py_trees.common.Access.READ)

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

    # calculate cube rotation speed in rad/s
    def calcTargetPosition(self):
        measurement_time = 0.1
        start = np.array([self.blackboard.cube_pos.y, self.blackboard.cube_pos.z])
        # table position relative to robot hardcoded
        center = np.array([-0.11587262153625, 0.89285403490067])
        time.sleep(measurement_time)
        end = np.array([self.blackboard.cube_pos.y, self.blackboard.cube_pos.z])

        center_start = (start - center)
        center_end = (end - center)

        cosine_angle = np.dot(center_start, center_end) / (np.linalg.norm(center_start) * np.linalg.norm(center_end))
        angle = np.arccos(cosine_angle)

        rads = angle/measurement_time

        arm_velocity = 0.1
        arm_position = np.array([self.blackboard.targetPosition.x, self.blackboard.targetPosition.y, self.blackboard.targetPosition.z])
        end3D = np.array([self.blackboard.cube_pos.x, self.blackboard.cube_pos.y, self.blackboard.cube_pos.z])
        distance_to_cube = np.linalg.norm(arm_position - end3D)
        travel_time = distance_to_cube/arm_velocity

        
        #offset = cube_velocity*travel_time
        #print("Offset: " + str(offset))
        
        predicted_angle = rads * travel_time
        rot_matrix = np.array([[np.cos(predicted_angle), -np.sin(predicted_angle)], [np.sin(predicted_angle), np.cos(predicted_angle)]])
        
        new_vect = np.dot(rot_matrix, center_end)
        predicted_point = new_vect + center

        new_position = Point(self.blackboard.cube_pos.x, predicted_point[0], predicted_point[1])
        #new_position = Point(self.blackboard.cube_pos.x, self.blackboard.cube_pos.y + offset, self.blackboard.cube_pos.z)
        new_orientation = normalizeAngles(Vector3(self.blackboard.cube_ori.x, self.blackboard.cube_ori.y, self.blackboard.cube_ori.z))
        return new_position, new_orientation

    def calcIntermediatePosition(self):
        measurement_time = 0.1
        p1 = np.array([self.blackboard.cube_pos.x, self.blackboard.cube_pos.y, self.blackboard.cube_pos.z])
        time.sleep(measurement_time)
        p2 = np.array([self.blackboard.cube_pos.x, self.blackboard.cube_pos.y, self.blackboard.cube_pos.z])

        distance = np.linalg.norm(p2-p1)
        cube_velocity = distance/measurement_time

        arm_velocity = 0.1
        arm_position = np.array([self.blackboard.targetPosition.x, self.blackboard.targetPosition.y, self.blackboard.targetPosition.z])
        distance_to_cube = np.linalg.norm(arm_position - p2)
        travel_time = distance_to_cube/arm_velocity

        offset = cube_velocity*travel_time

        new_position = basicPosition
        new_orientation = normalizeAngles(Vector3(self.blackboard.cube_ori.x, self.blackboard.cube_ori.y, self.blackboard.cube_ori.z))
        return new_position, new_orientation



    def update(self):
        """
        When is this called?
          Every time your behaviour is ticked.

        What to do here?
          - Triggering, checking, monitoring. Anything...but do not block!
          - Set a feedback message
          - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        """
        if self.name == "calcTargetPos":
            new_position, new_orientation = self.calcTargetPosition()
            self.blackboard.targetPosition = new_position
            self.blackboard.targetOri = new_orientation
        elif self.name == "calcIntermediatePos":
            new_position, new_orientation = self.calcIntermediatePosition()
            self.blackboard.targetPosition = new_position
            self.blackboard.targetOri = new_orientation
        elif self.name == "calcNeutralPos":
            self.blackboard.targetPosition = basicPosition
            self.blackboard.targetOri = basicOrienation
        elif self.name == "calcPlacingPos":
            self.blackboard.targetPosition = placingPosition
            self.blackboard.targetOri = placingOrientation
        else:
            self.blackboard.targetPosition = basicPosition
            self.blackboard.targetOri = basicOrienation

        print(self.name)
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        When is this called?
           Whenever your behaviour switches to a non-running state.
            - SUCCESS || FAILURE : your behaviour's work cycle has finished
            - INVALID : a higher priority branch has interrupted, or shutting down
        """
        self.logger.debug("  %s [Foo::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))
