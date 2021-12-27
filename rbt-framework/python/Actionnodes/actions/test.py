from geometry_msgs.msg import Vector3
import numpy as np

def normalizeAngle(offset, range, angle):
    new_angle = ((angle - (offset-range/2))%range) + (offset-range/2)
    return new_angle

def normalizeAngles(angles):
    print("Normalizing " +  str(angles))
    x = normalizeAngle(basicOrienation.x, 90, angles.x)
    y = normalizeAngle(basicOrienation.y, 90, angles.y)
    z = normalizeAngle(basicOrienation.z, 90, angles.z)
    print("...to " +  str(Vector3(x,y,z)))
    return Vector3(x,y,z)
basicOrienation = Vector3(0, 90, 180)
for x in range(0,91):
    normalizeAngles((Vector3(x, 0, 0)))