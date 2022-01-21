import time

import py_trees
import action
import numpy as np
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3

basicPosition = Point(0.4, 0, 0.4)
basicOrienation = Vector3(0, np.pi / 2, np.pi)
placingPosition = Point(0.1, 0.1, 0.5)
placingOrientation = Vector3(0, np.pi / 2, np.pi)


# offset = 0, range = 90, returns angles between 315 and 45 degrees
def normalizeAngle(offset, range, angle):
    new_angle = ((angle - (offset - range / 2)) % range) + (offset - range / 2)
    return new_angle

def normalizeAngles(angles):
    #print("Normalizing " + str(angles))
    x = normalizeAngle(basicOrienation.x, np.pi / 2, angles.x)
    y = normalizeAngle(basicOrienation.y, np.pi / 2, angles.y)
    z = normalizeAngle(basicOrienation.z, np.pi / 2, angles.z)
    #print("...to " + str(Vector3(x, y, z)))
    return Vector3(x, y, z)

def calcTravelTime(arm_position, end_position, arm_velocity):
    distance_to_cube = np.linalg.norm(arm_position - end_position)
    return distance_to_cube / arm_velocity


class Action(action.Action):
    def __init__(self, name):
        super(Action, self).__init__(name)
        self.blackboard = None
        self.name = name
        self.executed = False

    def setup(self):
        self.blackboard = py_trees.blackboard.Client(name="calcPos")
        self.blackboard.register_key(key="targetPosition", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="targetPosition", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="armVelocity", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="targetOri", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(key="cube_pos", access=py_trees.common.Access.READ)
        self.blackboard.register_key(key="cube_ori", access=py_trees.common.Access.READ)

    def initialise(self):
        return

    '''
    Calculate cube rotation speed in rad/s and returns the position of the cube based on the travel_time of the gripper
    '''
    def calcTargetPosition(self, travel_time):
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

        rads = angle / measurement_time

        print("Table velocity Rads/s = " + str(rads))
        predicted_angle = rads * travel_time
        rot_matrix = np.array(
            [[np.cos(predicted_angle), -np.sin(predicted_angle)], [np.sin(predicted_angle), np.cos(predicted_angle)]])

        new_vect = np.dot(rot_matrix, center_end)
        predicted_point = new_vect + center

        new_position = Point(self.blackboard.cube_pos.x, predicted_point[0], predicted_point[1])
        new_orientation = normalizeAngles(
            Vector3(self.blackboard.cube_ori.x, self.blackboard.cube_ori.y, self.blackboard.cube_ori.z))
        new_orientation.x = new_orientation.x - rads * travel_time
        return new_position, new_orientation

    '''
    Returns a position that is above the cube to avoid pushing it by positioning the gripper
    '''
    def calcIntermediatePosition(self):
        x_offset = 0.2
        arm_position = np.array(
            [self.blackboard.targetPosition.x, self.blackboard.targetPosition.y, self.blackboard.targetPosition.z])
        end_pos = np.array(
            [self.blackboard.cube_pos.x + x_offset, self.blackboard.cube_pos.y, self.blackboard.cube_pos.z])
        new_position, new_orientation = self.calcTargetPosition(calcTravelTime(arm_position, end_pos, self.blackboard.armVelocity))
        new_intermediate_position = Point(new_position.x + x_offset, new_position.y, new_position.z)
        return new_intermediate_position, new_orientation

    def update(self):
        if self.name == "calcTargetPos":
            arm_position = np.array(
                [self.blackboard.targetPosition.x, self.blackboard.targetPosition.y, self.blackboard.targetPosition.z])
            end_pos = np.array(
                [self.blackboard.cube_pos.x, self.blackboard.cube_pos.y, self.blackboard.cube_pos.z])
            new_position, new_orientation = self.calcTargetPosition(calcTravelTime(arm_position, end_pos, self.blackboard.armVelocity))
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

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        return
