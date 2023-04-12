#!/usr/bin/env python

import numpy as np
import rospy

import sympy as sp
import rospkg
import collections

def getTrefoil(tt,offset,slower,lim_x, lim_y, lim_z):
    x=(sp.sin((tt+slower*offset)/slower)+2*sp.sin(2*((tt+slower*offset)/slower))+3)/6; # in [0,1] approx
    y=(sp.cos((tt+slower*offset)/slower)-2*sp.cos(2*((tt+slower*offset)/slower))+3)/6;
    z=((-sp.sin(3*((tt+slower*offset)/slower)))+1.0)/2.0; # in [0,1] approx

    x=min(lim_x)+(max(lim_x)-min(lim_x))*x
    y=min(lim_y)+(max(lim_y)-min(lim_y))*y
    z=min(lim_z)+(max(lim_z)-min(lim_z))*z

    return [x, y, z]

#bbox has three elements: [hx, hy, hz] (bbox of size hx x hy x hz)
Drone = collections.namedtuple('Drone', ["name","bbox", "slower", "offset", "lim_x", "lim_y", "lim_z"])

zmin=1.0
zmax=1.8

all_drones=[     #"name",     "bbox",           "slower", "offset", "lim_x",    "lim_y",    "lim_z"
            Drone("obstacle2", [1.0, 0.8, 3.0], 5.5,       0.0,     [9.0, 13.0], [-2.0, 2.0],   [zmin,zmax])
            ]

all_drones.append(Drone("obstacle1", [0.8, 0.8, 0.5],  5.5,   0.0,   [-1.0, 1.0], [-1.0, 1.0],   [zmin,zmax]))

tmp = rospkg.RosPack()
pwd_package=tmp.get_path('rmader')
t=sp.symbols('t')
for i in range(len(all_drones)):
    drone_i=all_drones[i]
    traj=getTrefoil(t, drone_i.offset, drone_i.slower, drone_i.lim_x, drone_i.lim_y, drone_i.lim_z)
    # print traj
    name_file=pwd_package+f"/param/{drone_i.name}.yaml"
    f = open(name_file, "w")
    f.write("# DO NOT EDIT. RUN THE PYTHON FILE INSTEAD TO GENERATE THIS .yaml FILE \n")
    f.write("traj_x: "+str(traj[0])+"\n")
    f.write("traj_y: "+str(traj[1])+"\n")
    f.write("traj_z: "+str(traj[2])+"\n")
    f.write("bbox: "+str(drone_i.bbox)+"\n")
    f.close()
    print ("Writing to " + name_file)