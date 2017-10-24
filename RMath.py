
from geometry_msgs.msg import Point, Quaternion
from math import sqrt,sin,cos,atan2,radians,pi,floor

global PointSub
def PointSub(A, B):
    C=Point()
    C.x=A.x-B.x;
    C.y=A.y-B.y;
    C.z=A.z-B.z;
    return C

def PointDist(A,B):
    C=PointSub(A, B)
    return sqrt(C.x*C.x+C.y*C.y+C.z*C.z)

def PixelCoorToPQ(x,y,theta):
    scale=0.05
    PtX=x*scale
    PtY=y*scale
    r3=sin(radians(theta*0.5))
    r4=cos(radians(theta*0.5))
    if(r4<0):
        r4=-r4
        r3=-r3
    return [PtX,PtY,r3,r4]

def QuaternionToAngle(Quaternion):
    theta=atan2(Quaternion.z,Quaternion.w)
    return Wrap2Pi(theta*2)

def Wrap2Pi(angle_rad):
    angle_rad = angle_rad - 2 * pi * floor((angle_rad + pi) / (2 * pi));
    return angle_rad

'''
a=Point(1,2,3)
b=Point(4,5,6)
c=PointDist(a,b)
print(c)
'''