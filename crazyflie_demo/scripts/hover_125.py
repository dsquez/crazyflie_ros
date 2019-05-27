#!/usr/bin/env python

import rospy
import crazyflie
import time
import uav_trajectory

if __name__ == '__main__':
    rospy.init_node('test_high_level')

    cf1 = crazyflie.Crazyflie("cf1", "/vicon/cf1/cf1")
    cf2 = crazyflie.Crazyflie("cf2", "/vicon/cf2/cf2")
    cf5 = crazyflie.Crazyflie("cf5", "/vicon/cf5/cf5")

    swarm = [
                cf1
                , 
                cf2
                ,
                cf5
            ]
    
    for drone in swarm:
        drone.setParam("commander/enHighLevel", 1)
        drone.setParam("stabilizer/estimator", 2) # Use EKF
        drone.setParam("stabilizer/controller", 2) # Use mellinger controller

        # reset kalman
        drone.setParam("kalman/resetEstimation", 1)

    
    for drone in swarm:
        drone.takeoff(targetHeight = 0.5, duration = 4.0)
    time.sleep(5.0)

    #cf.goTo(goal = [0.0, 0.0, 1.0], yaw=0.2, duration = 2.0, relative = False)
    #time.sleep(4.0)

    #cf.goTo(goal = [0.0, 0.0, 0.0], yaw=0.5, duration = 5.0, relative = False)
    for drone in swarm:
        drone.land(targetHeight = 0.0, duration = 5.0)
    time.sleep(6.0)
    '''traj1 = uav_trajectory.Trajectory()
    traj1.loadcsv("takeoff.csv")

    traj2 = uav_trajectory.Trajectory()
    traj2.loadcsv("figure8.csv")

    #print(traj1.duration)
    for drone in swarm:
        drone.uploadTrajectory(0, 0, traj1)
        drone.uploadTrajectory(1, len(traj1.polynomials), traj2)

    for drone in swarm:
        #drone.startTrajectory(0, timescale=1.0)
        drone.takeoff(targetHeight=.5,duration=4.0)
    #time.sleep(traj1.duration * 2.0)
    time.sleep(6.0)

    
        #cf.startTrajectory(0, timescale=1.0)
    cf2.startTrajectory(0, timescale=1.0)
    time.sleep(traj1.duration * 1.5)

    cf1.startTrajectory(1, timescale=1.0)
    cf2.startTrajectory(1, timescale=1.0, reverse=True)
    #for drone in swarm:
     #   drone.startTrajectory(1, timescale=2.0)
    time.sleep(traj2.duration * 2.0)

    #cf.startTrajectory(1, timescale=2.0)
    #time.sleep(traj2.duration * 2.0)

    for drone in swarm:
        #drone.startTrajectory(0, timescale=1.0, reverse=True)
        drone.land(targetHeight=0.0,duration=6.0)
    time.sleep(7.0)'''

    #cf.startTrajectory(0, timescale=1.0, reverse=True)
    #time.sleep(traj1.duration * 1.0)

    for drone in swarm:
        drone.stop()
