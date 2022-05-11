import math

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

"""
!@brief pub_velocity             Robot velocity publisher.
"""
pub_velocity = rospy.Publisher("cmd_vel", Twist, queue_size=10)

"""
!@brief lidarPolarToPoints       converts point from polar to cartesian coordinate system
@param lidar_polar_data          data from lidar in polar coordinates
@return                          lidar data in cartesian coordinates
"""
def lidarPolarToPoints(lidar_polar_data: LaserScan):
    x0 = math.cos(lidar_polar_data.angle_min+(lidar_polar_data.angle_increment*0)) * lidar_polar_data.ranges[0]
    y0 = math.sin(lidar_polar_data.angle_min+(lidar_polar_data.angle_increment*0)) * lidar_polar_data.ranges[0]
    x1 = math.cos(lidar_polar_data.angle_min+(lidar_polar_data.angle_increment*1)) * lidar_polar_data.ranges[1]
    y1 = math.sin(lidar_polar_data.angle_min+(lidar_polar_data.angle_increment*1)) * lidar_polar_data.ranges[1]

    x2 = math.cos(lidar_polar_data.angle_min+(lidar_polar_data.angle_increment*2)) * lidar_polar_data.ranges[2]
    y2 = math.sin(lidar_polar_data.angle_min+(lidar_polar_data.angle_increment*2)) * lidar_polar_data.ranges[2]
    x3 = math.cos(lidar_polar_data.angle_min+(lidar_polar_data.angle_increment*3)) * lidar_polar_data.ranges[3]
    y3 = math.sin(lidar_polar_data.angle_min+(lidar_polar_data.angle_increment*3)) * lidar_polar_data.ranges[3]

    x4 = math.cos(lidar_polar_data.angle_max-(lidar_polar_data.angle_increment*1)) * lidar_polar_data.ranges[4]
    y4 = math.sin(lidar_polar_data.angle_max-(lidar_polar_data.angle_increment*1)) * lidar_polar_data.ranges[4]
    x5 = math.cos(lidar_polar_data.angle_max-(lidar_polar_data.angle_increment*0)) * lidar_polar_data.ranges[5]
    y5 = math.sin(lidar_polar_data.angle_max-(lidar_polar_data.angle_increment*0)) * lidar_polar_data.ranges[5]

    return [x0, y0, x1, y1, x4, y4, x5, y5, x2, y2, x3, y3]
""" 36 stopnie"""
"""
!@brief getNextDestination       Calculates the next robot goal in local coordinate system
                                 (it works like a carrot on the stick).
@param lidar_points              Points from lidar.
@return                          The destination coordinates.
"""
def getNextDestination(lidar_points):
    ax = ((lidar_points[0]) + lidar_points[6])/2
    ay = ((lidar_points[1]) + lidar_points[7])/2
    bx = ((lidar_points[2]) + lidar_points[4])/2
    by = ((lidar_points[3]) + lidar_points[5])/2

    dx = (ax + bx)/2
    dy = (ay + by)/2



    return dx, dy

"""
!@brief wallAtFront              Checks whether there is a wall in front of the robot.
@param lidar_points              Points from lidar.
@return                          True if the wall is present, false if the way is free.
8 - 11"""
def wallAtFront(lidar_points) -> bool:
    """if ( lidar_points[8] > 0,5 or lidar_points[9] > 0,5 or lidar_points[10] > 0,5 or lidar_points[11] > 0,5):
        return False
    else:
        return True"""
    return False



"""
!@brief PID                      Simple PID driver.
@param u                         Driver input signal.
@return                          Driver output signal.
"""
def PID(u: float) -> float:
    k = 1
    return k*u

"""
!@brief move                     Send the move forward command to the robot, with given
                                 additional rotation.
@param rotation                  Rotation speed, from -1 to 1.
"""
def move(rotation: float):
    twist = Twist()

    twist.linear.x = 1
    twist.angular.z = rotation

    pub_velocity.publish(twist)

    pass

"""
!@brief stop                     Send the stop command to the robot.
"""
def stop():
    twist = Twist()

    twist.linear.x = 0
    twist.angular.z = 0

    pub_velocity.publish(twist)
"""
@brief lidarCallback             Process lidar data.
@param lidar_polar_data          ROS lidar message.
"""
def lidarCallback(lidar_polar_data: LaserScan):
    lidar_points = lidarPolarToPoints(lidar_polar_data)
    rospy.loginfo(lidarPolarToPoints(lidar_polar_data))
    print("D: ", getNextDestination(lidar_points))
    print(wallAtFront(lidar_points))



    if wallAtFront(lidar_points):
        stop()
    else:
        move(getNextDestination(lidar_points)[1])

if __name__ == '__main__':
    rospy.init_node("wseiz_1")
    rate = rospy.Rate(10)
    sub_lidar = rospy.Subscriber("base_scan", LaserScan, lidarCallback, queue_size=10)

    while not rospy.is_shutdown():
        rate.sleep()
