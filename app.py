#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Aug 28 11:14:47 2022

@author: francesca
http://www.vergenet.net/~conrad/boids/pseudocode.html
https://www.youtube.com/watch?v=XiYnZ6Dy4v8
https://www.youtube.com/watch?v=VZ_tS4F6P2A
"""

# import random
# import matplotlib
import streamlit as st
import time
import numpy as np
from streamlit.hello.utils import show_code

from functions import move



def animation_demo():
    # %%
    L = 450
    tsteps = 500
    V0 = 20                     # Initial velocity
    
    wall_repulsion = V0          # https://vanhunteradams.com/Pico/Animal_Movement/Boids-algorithm.html#Screen-edges
                                # should be similar to max speed, otherwise they escape
    margin = 40
    max_speed = V0

    # %%
    n = st.sidebar.slider("Number of birds", min_value= 2, max_value = 1000, value = 300, step =2)

    
    # Rule 1
    # velocity to center contribution
    R = st.sidebar.slider("1. COHESION", 
                          min_value=0.,  max_value=2., step = 0.05, value = 0.1)
    
    # Rule 2
    bird_repulsion = st.sidebar.slider("2. SEPARATION", 
                                       min_value=0., max_value=10., step = 1., value = 7.)        
    # Rule 3
    match_velocity =  st.sidebar.slider("3. ALIGNMENT", 
                                       min_value=0., max_value=10., step = 0.5, value = 3.)     
    

    # Community distance (~ community size)
    neighbors_dist = st.sidebar.slider("Community distance i.e. distance of influence between birds (From spread to packed). Applied to rule 1 and rule 3)",
                                       min_value = 10, max_value=200, value = 70)

    privacy = st.sidebar.slider("Bird privacy (distance at which we apply rule 2)", 
                                       min_value=0, max_value=30, step = 1, value = 14)
    # low values make more "stains" of birds, nice visualization
    # When they see each other at this distance, 
    # they apply bird repulsion in the other direction

    progress_bar = st.sidebar.progress(0)

    frame_text = st.sidebar.empty()
    image = st.empty()
    

    

    
    # %% INITIALIZE
    x = np.zeros((n,tsteps))
    y = np.zeros((n,tsteps))
    
    x[:,0] = np.random.uniform(low=-L, high=L, size=(int(n),))
    y[:,0] = np.random.uniform(low=-L, high=L, size=(int(n),))
    
    x[:,1] = x[:,0] + np.random.uniform(low=-V0, high=V0, size=(int(n),))
    y[:,1] = y[:,0] + np.random.uniform(low=-V0, high=V0, size=(int(n),))
    
    
    y = y.astype(int)
    x = x.astype(int)
    
    # %% MOVE
    # ANIM = np.zeros((tsteps, L*2+margin, L*2+margin))
    N = np.zeros((L*2+margin, L*2+margin))
    last_col = np.shape(N)[0]-1
    for t in range(1,tsteps-1):
        x[:, t+1],y[:, t+1] = move(x[:, t],y[:, t], x[:,t-1],y[:,t-1],
                                   n, neighbors_dist, R, privacy, bird_repulsion, match_velocity, L, margin, wall_repulsion, max_speed)
        
        y = y.astype(int)
        x = x.astype(int)
        N = np.zeros((L*2+margin, L*2+margin))
        # N[y[:,t]+L+10, x[:,t]+L+10] = 0
        N[y[:,t+1]+L+round(margin/2), x[:,t+1]+L+round(margin/2)] = 1
        
        
        # Increase the size of the points
        to_plot = N
        to_plot[1:last_col,:] = N[1:last_col,:] + N[0:(last_col-1),:] # right shift of the matrix
        to_plot[:,1:last_col] = N[:,1:last_col] + N[:,0:(last_col-1)] # upper shift of the matrix
        to_plot[0:(last_col-1),:] = N[0:(last_col-1),:] + to_plot[1:last_col,:] # left shift of the matrix
        to_plot[:,0:(last_col-1)] = N[:,0:(last_col-1)] + to_plot[:,1:last_col]  # bottom shift of the matrix
        to_plot = np.ceil(to_plot/np.amax(to_plot))
        
        # matplotlib.use("agg")  # https://discuss.streamlit.io/t/mediafilemanager-missing-file-when-try-to-plot-two-line-chart-o-the-same-figure/5735/8
        # _lock = RendererAgg.lock
        # with _lock:

        # ANIM[t] = to_plot
        image.image(to_plot, use_column_width=True)
        time.sleep(0.05)

    # for t in range(1,tsteps-1):
    #     image.image(ANIM[t], use_column_width=True)
    #     time.sleep(0.05)

    # We clear elements by calling empty on them.
    progress_bar.empty()
    frame_text.empty()

    # Streamlit widgets automatically run the script from top to bottom. Since
    # this button is not connected to any other logic, it just causes a plain
    # rerun.
    st.button("Re-run")


st.set_page_config(page_title="Flocks", page_icon="🐤") #  📹
st.markdown("# Flock Animation")
st.sidebar.header("Parameters")
st.markdown(
    """The animation shows a simulation of a flock of birds. 
Flocks of birds follows three rules simulataneously:
1. __Cohesion__ rule: a bird flies towards the center of mass of the nearest neighbors.
    
2. __Separation__ rule: a bird tries to avoid collisions with other birds.

3. __Alignment__ rule: a bird tries to match velocity with near birds.

You can change the intensity of these rules moving the sliders in the lateral bar.

More details in my github page [https://frapria.github.io/blog/flocks](https://frapria.github.io/blog/flocks)
"""
)

animation_demo()

        
show_code(animation_demo)