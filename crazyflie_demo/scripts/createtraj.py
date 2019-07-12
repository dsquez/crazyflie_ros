#!/usr/bin/env python

###################################################################
######################MATLAB CODE##################################

'''
%% 1
% Calculate the trajectory of a drone given an initial velocity and
% direction
x0 = [0 0 0];
v0 = [10,10,10];
init = [ x0 v0 ];
tspan = [ 0 5 ];

[tout, yout] = ode45(@eomtraj,tspan,init);

for i = 1:length(yout(:,1))
	if yout(i,3) < 0
		yout(i,3) = 0;
	end
end

figure(1)
plot3(yout(:,1),yout(:,2),yout(:,3))
axis equal
hold on

index = find(yout(:,3)==0,2);
index = index(2);

newy = yout(1:index,1:3);

p = polyfitn(newy(:,1:2),newy(:,3),2);

model = polyvaln(p,newy(:,1:2));

plot3(newy(:,1),newy(:,2),model)



function xdot = eomtraj(t,x0)
m = 10;
rho = 1.23;
Cd = 0.4;
A = .01;
g = 9.81;

xdot = zeros(length(x0),1);
xi = x0(1);
yi = x0(2);
zi = x0(3);
vxi = x0(4);
vyi = x0(5);
vzi = x0(6);

xdot(1) = vxi;
xdot(2) = vyi;
xdot(3) = vzi;
xdot(4) = -rho*Cd*A*x0(4)^2/(2*m);
xdot(5) = -rho*Cd*A*x0(5)^2/(2*m);
xdot(6) = (-m*g - sign(x0(6))*.5*rho*Cd*A*x0(6)^2)/m;



end'''

#######################END MATLAB CODE##############################
####################################################################

import rospy
import crazyflie
import time
import uav_trajectory
import numpy as np
import scipy
from scipy.integrate import odeint
import math
import swarmlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def calctraj(x0,v0):
	
	init = np.append(x0,v0,axis=0)
	#print "init:", init
	tspan = np.linspace(0,4,num=2000)
	sol = odeint(eomtraj,init,tspan)
	positions = sol[:,0:3]
	return positions

def eomtraj(x0,t):
	m = 10
	rho = 1.23
	Cd = 0.4
	A = .01
	g = 9.81
	xdot = np.zeros([len(x0)])
	xi = x0[0]
	yi = x0[1]
	zi = x0[2]
	vxi = x0[3]
	vyi = x0[4]
	vzi = x0[5]
	xdot[0] = vxi
	xdot[1] = vyi
	xdot[2] = vzi
	xdot[3] = -rho*Cd*A*math.pow(x0[3],2)/(2*m)
	xdot[4] = -rho*Cd*A*math.pow(x0[4],2)/(2*m)
	xdot[5] = (-m*g - np.sign(x0[5])*.5*rho*Cd*A*math.pow(x0[5],2))/m
	return xdot




if __name__ == '__main__':

	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	#ax2 = fig.add_subplot(122)

	print "program started"
	rospy.init_node('test_high_level')

	#cf1 = crazyflie.Crazyflie("cf1", "/vicon/cf1/cf1")
	cf2 = crazyflie.Crazyflie("cf2", "/vicon/cf2/cf2")
	print "crazyflies initialized"
	swarm = [
				cf2
				#, 
				#cf2
			]
	cf_name = 'cf2'
	droneob = swarmlib.Drone(cf_name)

	for drone in swarm:
		drone.setParam("commander/enHighLevel", 1)
		drone.setParam("stabilizer/estimator", 2) # Use EKF
		drone.setParam("stabilizer/controller", 2) # Use mellinger controller

		# reset kalman
		drone.setParam("kalman/resetEstimation", 1)
	print "drone parameters set"

	height = 0.5
	waittime = 0.75
	rate = rospy.Rate(35)


	for drone in swarm:
		drone.takeoff(targetHeight = height, duration = 4.0)
		print "takeoff command given"
	time.sleep(5.0)

	for drone in swarm:
		print "dronepos:", drone.position()
		traj = calctraj(drone.position(),np.array([2,0,3]))

	#print "traj:",traj
	count = 0

	for waypoint in range(len(traj[:,0])):
		if traj[waypoint,2] >= height / 5:
			count += 1
			droneob.sp = traj[waypoint,:] + [0,0,0]
			print 'fly to', droneob.sp
			droneob.fly()
			#cf2.goTo(goal = traj[waypoint,:] + [0,0,0],yaw=0.0,duration=waittime,relative=False)
			#print "goalpos", traj[waypoint,:]
			#time.sleep(waittime)
			rate.sleep()
		else:
			break

	#time.sleep(3.0) 
	tspan = np.linspace(0,4,num=500)
	ax.plot(traj[0:count,0], traj[0:count,1], traj[0:count,2])
	
	

	rate = rospy.Rate(10)
	drone_landing_pose = droneob.position()
	
	while drone_landing_pose[2] > -.1:
		droneob.sp = drone_landing_pose
 		drone_landing_pose[2] = drone_landing_pose[2]-0.005
 		print "Landing", drone_landing_pose
 		droneob.fly()
 		rate.sleep()
    

 	print 'reached the floor, shutdown'   	
  	time.sleep(0.5)
   	
  	#rospy.signal_shutdown('landed')
  	#rate.sleep()
  	
  	cf2.stop()
  	plt.show()
	#cf.goTo(goal = [0.0, 0.0, 1.0], yaw=0.2, duration = 2.0, relative = False)
	#time.sleep(4.0)

	#cf.goTo(goal = [0.0, 0.0, 0.0], yaw=0.5, duration = 5.0, relative = False)
	#for drone in swarm:
	#	drone.land(targetHeight = 0.0, duration = 3.0)
	#time.sleep(3.3)

	#for drone in swarm:
	#	drone.stop()













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

	
