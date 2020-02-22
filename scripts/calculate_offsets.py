#!/usr/bin/env python

from subprocess import call

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from force_torque_sensor.srv import CalculateSensorOffset
from force_torque_sensor.srv import SetSensorOffset
from geometry_msgs.msg import Wrench

import dynamic_reconfigure.client

def calculate_sensor_offsets():
    
    rospy.init_node('calculate_offsets')
    
    package_to_store = rospy.get_param('~package_to_store')
    store_to_file = rospy.get_param('~store_to_file')
    scenario = rospy.get_param('~scenario')
    robot = rospy.get_param('~robot')
    set_calculated_offset = rospy.get_param('~set_calculated_offset')
    offset_params_ns = rospy.get_param('~offset_params_ns', "/temp") # /arm/driver
    recalibrate_srv_ns = rospy.get_param('~recalibrate_srv_ns', "") # /arm/driver
    if robot == "kuka":
        joint_names = rospy.get_param('/controller_joint_names')
        controller_topic = '/position_trajectory_controller/command'
        calcOffset_service = '/CalculateOffsets'
        setOffset_service = '/SetSensorOffset'
    elif robot == "ur":
        joint_names = rospy.get_param("/hardware_interface/joints")
        controller_topic = "/pos_based_pos_traj_controller/command"
        calcOffset_service = '/CalculateOffsets'
        setOffset_service = '/SetSensorOffset'
    else:
        joint_names = rospy.get_param('/arm/joint_names')
        controller_topic = '/arm/joint_trajectory_controller/command'
        calcOffset_service = '/arm/CalculateOffsets'
        setOffset_service = '/arm/SetSensorOffset'

    trajectory_pub = rospy.Publisher(controller_topic, JointTrajectory, latch=True, queue_size=1)
    calculate_offsets_srv = rospy.ServiceProxy(calcOffset_service, CalculateSensorOffset)
    set_offsets_srv = rospy.ServiceProxy(setOffset_service, SetSensorOffset)
    
    ##print call('rospack find force_torque_sensor', shell=True)
    ##call('rosparam dump -v `rospack find force_torque_sensor`/config/sensor_offset.yaml /fts/Offset')
    
    # Posees
    poses = [[0.0, 0.0, 1.5707963, 0.0, -1.5707963, 0.0],
             [0.0, 0.0, 1.5707963, 0.0, 1.5707963, 0.0]]

    poses_kuka = [[0.0, -1.5707963, 1.5707963, 0.0, -1.5707963, 0.0],
             [0.0, -1.5707963, 1.5707963, 0.0, 1.5707963, 0.0]]
    
    poses_ur = [[1.5707963, -1.5707963, 1.5707963, -1.5707963, 1.5707963, 0.0],
                [1.5707963, -1.5707963, 1.5707963, -1.5707963, -1.5707963, 0.0]]

    measurement = Wrench()
    
    for i in range(0,len(poses)):
        trajectory = JointTrajectory()
        point = JointTrajectoryPoint()
        trajectory.header.stamp = rospy.Time.now()
        trajectory.joint_names = joint_names
        
        point.time_from_start = rospy.Duration(2.5)
        if robot == "kuka":
            point.positions = poses_kuka[i]
        elif robot == "ur":
            point.positions = poses_ur[i]
        else:
            point.positions = poses[i]
        
        trajectory.points.append(point)        
        trajectory_pub.publish(trajectory)            
        rospy.loginfo("Going to position: " + str(point.positions))
                
        rospy.sleep(10.0)
        
        rospy.loginfo("Calculating offsets.")
        ret = calculate_offsets_srv(False)

        measurement.force.x += ret.offset.force.x
        measurement.force.y += ret.offset.force.y
        measurement.force.z += ret.offset.force.z
        measurement.torque.x += ret.offset.torque.x
        measurement.torque.y += ret.offset.torque.y
        measurement.torque.z += ret.offset.torque.z


    measurement.force.x /= len(poses)
    measurement.force.y /= len(poses)
    measurement.force.z /= len(poses)
    measurement.torque.x /= len(poses)
    measurement.torque.y /= len(poses)
    measurement.torque.z /= len(poses)


    rospy.set_param(offset_params_ns + '/Offset/force/x', measurement.force.x)
    rospy.set_param(offset_params_ns + '/Offset/force/y', measurement.force.y)
    rospy.set_param(offset_params_ns + '/Offset/force/z', measurement.force.z)
    rospy.set_param(offset_params_ns + '/Offset/torque/x', measurement.torque.x)
    rospy.set_param(offset_params_ns + '/Offset/torque/y', measurement.torque.y)
    rospy.set_param(offset_params_ns + '/Offset/torque/z', measurement.torque.z)

    if set_calculated_offset:
        ret = set_offsets_srv(measurement)
        rospy.loginfo(ret.message)
        #client = dynamic_reconfigure.client.Client(recalibrate_srv_ns)
        #client.update_configuration({"force":{"x":measurement.force.x, "y":measurement.force.y, "z":measurement.force.z}, "torque":{"x":measurement.torque.x, "y":measurement.torque.y, "z":measurement.torque.z}})

    if store_to_file:
        if scenario == '':
            call('rosparam dump -v `rospack find ' + package_to_store + ' `/config/sensor_offset.yaml  '  + offset_params_ns +  '/Offset', shell=True)
        else:
            call('rosparam dump -v `rospack find ' + package_to_store + ' `/config/robot_with_' + scenario + '_offset.yaml  '  + offset_params_ns +  '/Offset', shell=True)



if __name__ == "__main__":
    
    try:
        calculate_sensor_offsets()
    except rospy.ROSInterruptException:
        pass
