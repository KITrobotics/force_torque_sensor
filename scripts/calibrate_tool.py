#!/usr/bin/env python

from subprocess import call
from math import sqrt

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from force_torque_sensor.srv import CalculateAverageMasurement
from geometry_msgs.msg import Vector3


def calibrate_tool():
    
    rospy.init_node('calibrate_tool')    
    
    tool_name = rospy.get_param('~tool_name')
    robot_name = rospy.get_param('~robot_name')
    store_to_file = rospy.get_param('~store_to_file')
    robot = rospy.get_param('~robot')

    if robot == "kuka":
        joint_names = rospy.get_param('/controller_joint_names')
        controller_topic = '/position_trajectory_controller/command'
        calcOffset_service = '/CalculateAverageMasurement'
    else:
        joint_names = rospy.get_param('/arm/joint_names')
        controller_topic = '/arm/joint_trajectory_controller/command'
        calcOffset_service = '/arm/CalculateOffsets'

    trajectory_pub = rospy.Publisher(controller_topic, JointTrajectory, latch=True, queue_size=1)
    average_measurements_srv = rospy.ServiceProxy(calcOffset_service, CalculateAverageMasurement)

    # Posees
    poses = [[0.0, 0.0, 1.5707963, 0.0, -1.5707963, 0.0],
                   [0.0, 0.0, 1.5707963, 0.0, 1.5707963, 0.0],
                   [0.0, 0.0, 0.0, 0.0, 1.5707963, 0.0]]

    poses_kuka = [[0.0, -1.5707963, 1.5707963, 0.0, -1.5707963, 0.0],
                   [0.0, -1.5707963, 1.5707963, 0.0, 1.5707963, 0.0],
                   [0.0, -1.5707963, 1.5707963, 0.0, 0.0, 0.0]]

    measurement = []
    
    for i in range(0,len(poses)):  
        trajectory = JointTrajectory()
        point = JointTrajectoryPoint()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.joint_names = joint_names
        
        point.time_from_start = rospy.Duration(5.0)
        if robot == "kuka":
            point.positions = poses_kuka[i]
        else:
            point.positions = poses[i]
        
        trajectory.points.append(point)
        trajectory_pub.publish(trajectory)
        rospy.loginfo("Going to position: " + str(point.positions))
        
        rospy.sleep(10.0)
        
        rospy.loginfo("Calculating tool force.")
        #ret = average_measurements_srv(500, 0.01)
        ret = average_measurements_srv(500, 0.01, "fts_base_link")

        measurement.append(ret.measurement)

    CoG = Vector3()

    Fg = (abs(measurement[0].force.z) + abs(measurement[1].force.z))/2.0;
    CoG.z = (sqrt(measurement[2].torque.x*measurement[2].torque.x + measurement[2].torque.y*measurement[2].torque.y)) / Fg;
    #CoG.z = (measurement[2].torque.x) / Fg;

    rospy.loginfo("Setting parametes for tool: " + tool_name)

    rospy.set_param('/temp/tool/CoG_x', CoG.x)
    rospy.set_param('/temp/tool/CoG_y', CoG.y)
    rospy.set_param('/temp/tool/CoG_z', CoG.z)
    rospy.set_param('/temp/tool/force', Fg)
    
    if store_to_file:
      call("rosparam dump -v `rospack find iirob_description`/tools/urdf/" + tool_name + "/" + robot_name + "_gravity.yaml /temp/tool", shell=True)


if __name__ == "__main__":
    
    try:
        calibrate_tool()
    except rospy.ROSInterruptException:
        pass
