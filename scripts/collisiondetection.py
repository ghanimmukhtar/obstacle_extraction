#!/usr/bin/python
# -*- coding: utf-8 -*-

""" Alexandre Coninx
    ISIR CNRS/UPMC
    15/03/2017
""" 

import numpy as np

import csv


#import mttkinter as Tkinter

import matplotlib

matplotlib.use("GTKAgg")

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

plt.ion()

from itertools import product

default_archives = "demo_trj.dat"
#This obstacle is (just) good for traj 0 and bad for traj 2
test_obstacle = ((0.5,0.4,0.2),(1,1.4,0.8))

#Points ordering in faces
faces_plottable = ((0,1,3,2,0),
		 (0,1,5,4,0),
		 (0,4,6,2,0),
		 (4,5,7,6,4),
		 (5,7,3,1,5),
		 (7,3,2,6,7))

faces_by_corner = tuple((f[0],f[2]) for f in faces_plottable)



def load_file(fname=default_archives):
	trajectories = list()
	with open(fname,'r') as fd:
		reader = csv.reader(fd,delimiter=' ')
		for line in reader:
			index, n_arm, n_ball, dim = [int(field) for field in line[:4]]
			#print("Trajectory loaded: index = %d n_arm = %d n_ball = %d" % (index, n_arm, n_ball))
			assert (len(line) - 1) == (4 + (n_arm + n_ball)*3 + 2*7), "Trajectory has wrong number of entries"
			arm_index = 4
			ball_index = arm_index + dim*n_arm
			joints_release_pos_index = ball_index + dim*n_ball
			joints_release_speed_index = joints_release_pos_index+7
			arm = np.array([float(field) for field in line[arm_index:ball_index]]).reshape((dim,n_arm)).T
			ball = np.array([float(field) for field in line[ball_index:joints_release_pos_index]]).reshape((dim,n_ball)).T
			joints_release_pos = np.array([float(field) for field in line[joints_release_pos_index:joints_release_speed_index]])
			joints_release_speed = np.array([float(field) for field in line[joints_release_speed_index:-1]])
			trajectories.append((index,arm,ball,joints_release_pos,joints_release_speed))
	print("%d trajectories loaded" % len(trajectories))
	return trajectories
			

def project_along_axis(point,axis):
	return tuple(point[i] for i in range(len(point)) if i != axis)

def get_common_dims(a,b):
	return tuple(i for i in range(len(a)) if a[i]==b[i])

def check_segment_plane_collision(p1,p2,plane_corners):
	(c1, c2) = plane_corners
	cd = get_common_dims(c1,c2)
	assert(len(cd)==1)
	dim_plan = cd[0]
	pos_plan = c1[dim_plan]
	if((p1[dim_plan] < pos_plan) == (p2[dim_plan] < pos_plan)): # same side - no collision possible
		return False
	else: #find intersection
		dp_coord_from_p1 = (pos_plan-p1[dim_plan])/(p2[dim_plan]-p1[dim_plan])
		(p1p, p2p, c1p, c2p) = tuple(np.array(project_along_axis(x,dim_plan)) for x in (p1,p2,c1,c2))
		p_intersect=p1p + dp_coord_from_p1*(p2p - p1p)
		for dim in zip(c1p,c2p,p_intersect):
			if((dim[2] < np.min(dim[:2])) or (dim[2] > np.max(dim[:2]))):
				return False # out of the rectangle
		return True	
		

def compute_box(p1,p2):
	points = tuple(product(*zip(p1,p2)))
	return points

def get_faces_by_corners(points):
	return tuple(tuple(points[i] for i in f) for f in faces_by_corner)

def draw_box(ax,boxpoints):
	for f in faces_plottable:
		toplot = list([boxpoints[i] for i in f])
		ax.plot3D(*zip(*toplot),color='orange')
			
def plot_traj(traj,obstacles=None):
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	arm = traj[1]
	ball = traj[2]
	ax.plot(arm[:,0],arm[:,1],arm[:,2],color='black')
	ax.plot(ball[:,0],ball[:,1],ball[:,2],color='blue')
	good = True
	minrange = np.inf
	if(obstacles):
		for o in obstacles:
			points = compute_box(*o)
			draw_box(ax,points)
		badindices, minrange = check_trajectoire_boxes_collision(ball,obstacles)
		if(badindices):
			for i in badindices:
				ax.plot((ball[i,0],ball[i+1,0]),(ball[i,1],ball[i+1,1]),(ball[i,2],ball[i+1,2]),color='red',lw=4)
			good = False
	plt.title("Trajectory %s; mindist = %f" %("good" if good else "bad", minrange))
	return good, minrange



def plot_multiple_trajs(archive,indices,obstacles=None):
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	if(obstacles):
		for o in obstacles:
			points = compute_box(*o)
			draw_box(ax,points)
	goods = list()
	for (i,traj) in enumerate(archive):
		if indices and i not in indices:
			continue
		arm = traj[1]
		ball = traj[2]
		good = True
		if(obstacles):
			badindices, minrange = check_trajectoire_boxes_collision(ball,obstacles)
			if(badindices):
				for i in badindices:
					ax.plot((ball[i,0],ball[i+1,0]),(ball[i,1],ball[i+1,1]),(ball[i,2],ball[i+1,2]),color='red',lw=4)
				good = False
		if(good):
			goods.append(i)
		ax.plot(arm[:,0],arm[:,1],arm[:,2],color='black')
		ax.plot(ball[:,0],ball[:,1],ball[:,2],color=('blue' if good else 'yellow'))
	return goods

def distance(p1,p2):
	return np.sqrt(np.sum((np.array(p2)-np.array(p1))**2))

def compute_com(obs): # center of mass
	p1,p2 = obs
	return tuple(0.5*(np.array(p1) + np.array(p2)))

def very_crude_distance_to_obstacle(p,obspoints,com):
	''' Min of distance to each point and to center of mass -_- '''
	dists = list(distance(p,op) for op in (obspoints + (com,)))
	return np.min(dists)


def eval_trajs(archive,obstacles=None):
	goods = list()
	bads = list()
	mindists = list()
	for (i,traj) in enumerate(archive):
		ball = traj[2]
		good = True
		minrange = np.inf
		if(obstacles):
			badindices, minrange = check_trajectoire_boxes_collision(ball,obstacles)
			if(badindices):
				good = False
		if(good):
			goods.append(i)
		else:
			bads.append(i)
		mindists.append(minrange)
	ordering = np.argsort(-np.array(mindists))
	sorted_goods = [i for i in ordering if i in goods]
	sorted_good_dists = [mindists[i] for i in sorted_goods]
	return sorted_goods, sorted_good_dists, bads

def plot_traj_2d(traj):
	arm = traj[1]
	ball = traj[2]
	fig = plt.figure()
	ax = fig.add_subplot(111)
	ax.plot(arm[:,0],arm[:,1],color='black')
	ax.plot(ball[:,0],ball[:,1],color='blue')


def check_segment_box_collision(p1, p2, boxfacesbycorners):
	for face in boxfacesbycorners:
		if(check_segment_plane_collision(p1,p2,face)):
			return True
	return False

def check_trajectoire_boxes_collision(ballpoints,obstacles):
	badindices = []
	minrange = np.inf
	for o in obstacles:
		boxpoints = compute_box(*o)
		com_obs = compute_com(o)
		facesbycorners = get_faces_by_corners(boxpoints)
		for i in range(len(ballpoints)-1):
			if(check_segment_box_collision(ballpoints[i],ballpoints[i+1],facesbycorners)):
				badindices.append(i)
			d = very_crude_distance_to_obstacle(ballpoints[i],boxpoints,com_obs)
			if(d < minrange):
				minrange = d
	return badindices, minrange


def test_system():
	trajectories = load_file() # Load trajectories
	obstacles = (test_obstacle,) # Test obstacle
	#obstacles = () # No obstacle case
	sorted_goods, sorted_good_dists, bads = eval_trajs(trajectories,obstacles) # Evaluate the trajectories on the obstacles
	best = sorted_goods[0] # Choose the best (further from obstacles according to the crude distance)
	print("Choosing trajectory #%d (crude min dist %f)" % (best, sorted_good_dists[0]))
	plot_traj(trajectories[best],obstacles) # Plot the best
	plt.show()

if(__name__ == '__main__'):
	test_system()
