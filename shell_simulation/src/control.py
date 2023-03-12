#!/usr/bin/env python3 
import numpy as np 
import rospy 
from std_msgs.msg import Float64MultiArray, Float32 , Float64
from geometry_msgs.msg import Quaternion
from  sensor_msgs.msg import NavSatFix
#from carla_msgs.msg import CarlaEgoVehicleStatus


rospy.init_node('control_node')


class Get :
    def position(sat : NavSatFix ) -> np.array:
        pose = [sat.longitude,sat.latitude,sat.altitude]
        return np.array(pose)
    def speed(s : Float32) :
        speed = s.data
        return speed
    def orientaion () :
        theta = 0.0
        return theta
    def path () :
        path = 0.0
        return np.array(path)
    
rospy.Subscriber("/carla/ego_vehicle/gnss",NavSatFix,Get.position)
rospy.Subscriber("/carla/ego_vehicle/speedometer",Float32,Get.speed)
#rospy.Subscriber("/carla/ego_vehicle/vehicle_status",CarlaEgoVehicleStatus,Get.orentaion)
#rospy.Subscriber("path",location,Get.path) 
# above are comments as we dont have carla msgs  or path node msgs

throttle_publish = rospy.Publisher("/throttle_command",Float64 , queue_size=10)
breaks_publish = rospy.Publisher("/break_command",Float64 , queue_size=10)
steer_publish = rospy.Publisher("/steering_command",Float64 , queue_size=10)

vel_des = rospy.get_param('vel_des')
kp = rospy.get_param('kp')
ki = rospy.get_param('ki')
kd = rospy.get_param('kd')
k1 = rospy.get_param('k1')
ks = rospy.get_param('ks')
rate = rospy.get_param('rate')
mode = rospy.get_param('mode')

class Controller :
    def __init__(self,L=1) -> None:
        self.L=L
    def throttle_control (self,vel_current) :
        #global kp,ki,kd,rate, vel_des , prev_vel_err ,i
        vel_err =vel_des - vel_current
        delta_err = vel_err - prev_vel_err
        dt = 1/rate
        prev_vel_err = vel_err
        p = kp * vel_err
        i = i + ki * vel_err * dt
        d = kd * (delta_err/dt)
        PID =  p + i + d
        throttle = np.tanh(PID)
        return throttle
    def steer_control (self,mode,state,vel_current,waypoint) :
        """
        state is [Xcar , Ycar ,Zcar , THETAcar]
        waypoint is [X waypoint , Y waypoint , Z waypoint , THETA waypoint ]
        """
        global k1 , ks 
        if mode is "p" :
            del_x = waypoint [0] - state [0]
            del_y = waypoint [1] - state [1]
            alpha = np.arctan2(del_y , del_x) - state [3]
            id = np.sqrt(del_x**2 + del_y**2)
            steer = np.arctan2((2*self.L* np.sin(alpha)),id)            
        elif mode is "s" :
            del_x = waypoint [0] - state [0]
            del_y = waypoint [1] - state [1]
            alpha = np.arctan2(del_y , del_x) - state [3]
            id = np.sqrt(del_x**2 + del_y**2)
            ct_err = id * np.sin(alpha)
            epthi = waypoint [3] - state [3]
            steer = epthi + np.arctan2((k1*ct_err) , (vel_current + ks))
        elif mode is "lqr" :
            pass
        return steer
    
controller = Controller()


while not rospy.is_shutdown() :
    #getting input parameters #not sure of the formate
    poition = Get.position()
    theta = Get.orientaion()
    velocity = Get.speed()
    waypoint = Get.path()

    #calculating and casting outputs
    throttle64 = Float64(Controller.throttle_control())
    steer64 = Float64(Controller.steer_control())

    #publishing
    if (throttle64 > 0) :
        throttle_publish.publish(throttle64)
    else :
        throttle_publish.publish(throttle64)
    steer_publish.publish(steer64)