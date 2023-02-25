#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Aug 27 17:53:41 2022

@author: francesca
"""
import numpy as np
from scipy.spatial.distance import pdist, squareform
import math

def moveToCenter(x0, y0, neighbors_dist, n, R):
    # Rule 1: Boids try to fly towards the centre of mass of neighbouring boids.
    # ABSOLUTE CENTER OF MASS: If center of mass consider the bird itself
    # center_x = np.mean(x0)
    # center_y = np.mean(y0)
    
    # Consider only the nearest neighbors (within neighbors_dist distance)
    m = squareform(pdist(np.transpose([x0,y0])))
    idx = (m<=neighbors_dist) # & (m!=0)
    
    center_x = np.zeros(n)
    center_y = np.zeros(n)
    vx = np.zeros(n)
    vy = np.zeros(n)
    for i in range(0, n-1):
        center_x[i] = np.mean(x0[idx[i,]])
        center_y[i] = np.mean(y0[idx[i,]])
        vx[i] = -(x0[i] - center_x[i])*R
        vy[i] = -(y0[i] - center_y[i])*R
    
    # RELATIVE CENTER OF MASS:
    # Remove from the average the contribute of the bird considered
    # It is a relative center of mass, not absolute.
    # relative_center_x = ( np.mean(x0) - x0/len(x0) )*( len(x0)/( len(x0)-1) )
    # relative_center_y = ( np.mean(y0) - y0/len(y0) )*( len(y0)/( len(y0)-1) )
    

    return vx,vy



def avoidOthers(x0, y0, n, privacy, bird_repulsion):
    # Rule 2: Boids try to keep a small distance away from other objects (including other boids).
    # dist = np.triu(squareform(pdist(np.transpose([x0,y0]))))
    dist = squareform(pdist(np.transpose([x0,y0])))
    
    idxmat = (dist<privacy) & (dist !=0)
    idx = np.transpose(np.array(np.where(idxmat)))
    
    vx = np.zeros(n)
    vy = np.zeros(n)
    

    vx[idx[:,0]] = (x0[idx[:,0]] - x0[idx[:,1]])*bird_repulsion
    vy[idx[:,0]] = (y0[idx[:,0]] - y0[idx[:,1]])*bird_repulsion
    
    # x0[idx[:,0]] =    2*x0[idx[:,0]] - x0[idx[:,1]]
    # y0[idx[:,0]] =    2*y0[idx[:,0]] - y0[idx[:,1]]
    
    return vx,vy
    
    
def matchVelocities(x_prev, y_prev, x0, y0, n, neighbors_dist, match_velocity):
    # Rule 3: Boids try to match velocity with near boids.
    
    # Consider only the nearest neighbors (within neighbors_dist distance)
    m = squareform(pdist(np.transpose([x_prev,y_prev])))
    idx = (m<=neighbors_dist) # & (m!=0)
    
    vmean_x = np.zeros(n)
    vmean_y = np.zeros(n)
    for i in range(0, n-1):
        vmean_x[i] = np.mean( x0[idx[i,]] - x_prev[idx[i,]] )
        vmean_y[i] = np.mean( y0[idx[i,]] - y_prev[idx[i,]] )
        
    # vmean_x = np.mean(x1-x0)
    # vmean_y = np.mean(y1-y0)
    
    return vmean_x*match_velocity, vmean_y*match_velocity



# %%

def move(x0,y0, x_prev, y_prev, n, neighbors_dist, R, privacy, bird_repulsion, match_velocity, L, margin, wall_repulsion, max_speed):
    
    vx1,vy1 = moveToCenter(x0,y0, neighbors_dist, n, R)
    vx2,vy2 = avoidOthers(x0,y0,  n, privacy, bird_repulsion)
    vx3,vy3 = matchVelocities(x_prev, y_prev, x0,y0,  n, neighbors_dist, match_velocity)
    
    # wall_repulsion_x = 1/abs(x0 - L)
    # wall_repulsion_y = 1/abs(y0 - L)
    
    # vx = vx1 + vx2 + vx3
    # vy = vy1 + vy2 + vy3
    vx = x0-x_prev + vx1 + vx2 + vx3 
    vy = y0-y_prev + vy1 + vy2 + vy3 
    
    
    # max speed limit
    # Matrix 2xn. Get the length of the velocity vector for each boid, and 
    # scale it with the maximum value
    v_norm = np.zeros((2,n))
    v_vector = np.array([vx,vy])
    norm = np.linalg.norm(v_vector, axis=0)
    # v_norm = v_vector/)*max_speed
    v_norm[:, norm!=0] =  v_vector[:, norm!=0]/norm[norm!=0]*max_speed
    
    vx = v_norm[0,:]
    vy = v_norm[1,:]
    
    
    right_border_dist  = L - x0
    left_border_dist   = x0 + L
    upper_border_dist  = L - y0
    bottom_border_dist = y0 + L
    # dev_right_wall = deviate_from_walls(vx, vy, [1,0], right_border_dist, margin )
    
    vx[right_border_dist < margin] = vx[right_border_dist < margin] - wall_repulsion
    vx[left_border_dist < margin] = vx[left_border_dist < margin] + wall_repulsion
    vy[upper_border_dist < margin] = vy[upper_border_dist < margin] - wall_repulsion
    vy[bottom_border_dist < margin] = vy[bottom_border_dist < margin] + wall_repulsion
    
    x1 = x0 + vx
    y1 = y0 + vy
    
    x1 = np.round(x1) # errore di arrotondamento, se non lo faccio tutto si allinea su x=0, y=0
    y1 = np.round(y1)
    
    # reverse coordinates when hits a wall    
    
    # right_border_dist  = L - x1
    # left_border_dist   = x1 - L
    # upper_border_dist  = L - y1
    # bottom_border_dist = y1 - L
    
    # idx_x_right = x1 >= L - margin
    # idx_x_left = x1 <= -L + margin
    # idx_y_top =  y1 >= L - margin
    # idx_y_bottom = y1 <= -L + margin
    

    # idx_x = (x1 <= -L + margin) | (x1 >= L - margin) 
    # idx_y = (y1 <= -L + margin) | (y1 >= L - margin) 
    # x1[idx_x] = x0[idx_x] -vx[idx_x]*wall_repulsion
    # y1[idx_y] = y0[idx_y] -vy[idx_y]*wall_repulsion
    
    return x1,y1




# %%
def deviate_from_walls(vv_x, vv_y, perpendicular, d, margin, n):
    vv = [vv_x, vv_y]
    
    scalar_prod = np.sum(np.transpose(vv)*perpendicular, axis = 1)/np.linalg.norm(vv,axis=0)
    alpha0 = np.arccos( scalar_prod  ) # It's in radiants. Use math.pi
    alpha0[vv_y<0] = -alpha0[vv_y<0]
    
    # If angle between 0 and 180 degrees, increase the angle
    # If angle between 180 and 360, decrease the angle
    increase_angle =  (alpha0 >=0) & (alpha0< math.pi)
    increase_angle = np.asarray(increase_angle)*1
    increase_angle[increase_angle == 0] = -1
    
    
    # New angle must increase linearly with the proximity of the wall
    # (ramp function)
    intercept = alpha0
    slope = increase_angle*(math.pi / 2 - alpha0)/margin
    
    new_angle = np.zeros(n)
    # new_angle = (d > margin) if alpha0 else d*slope + intercept 
    new_angle[d>margin]= alpha0[d>margin]
    new_angle[d<=margin] = d[d<=margin] * slope[d<=margin] + intercept[d<=margin] 
    
    new_vvx = np.cos(new_angle)*np.linalg.norm(vv,axis=0)
    new_vvy = np.sin(new_angle)*np.linalg.norm(vv,axis=0)
    
    dev_vvx = np.round(new_vvx - vv_x, 3)
    dev_vvy = np.round(new_vvy - vv_y, 3)
    # sign = np.sign(np.cross([xx,yy, np.zeros(n)], [perpendicular, np.zeros(n) ])[2])
    
    return dev_vvx, dev_vvy