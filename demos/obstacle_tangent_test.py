#!/usr/bin/env python
"""Tangent line obstacle avoidance experiment."""

# for python3 compat
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import sys

INF = float('inf')

# Note: Adjust this depending on where this file ends up.
sys.path.append('..')
from aiboids.point2d import Point2d
from aiboids.vehicle2d import SimpleVehicle2d, SimpleObstacle2d
from aiboids.steering import WaypointPath

######################################################
# Duck punching!
def vehicledata(veh):
    return [veh.pos.x, veh.pos.y, veh.vel.x, veh.vel.y]

def vehiclelog_init(veh, pt=None):
    veh.xdata = list()
    veh.ydata = list()
    veh.vxdata = list()
    veh.vydata = list()

    if pt:
        veh.obs_center = pt
        veh.dsqdata = list()
SimpleVehicle2d.vehiclelog_init = vehiclelog_init

def vehiclelog(veh):
    veh.xdata.append(veh.pos.x)
    veh.ydata.append(SCREEN_SIZE[1] - veh.pos.y)
    veh.vxdata.append(veh.vel.x)
    veh.vydata.append(-veh.vel.y)
    try:
        veh.dsqdata.append((veh.pos - veh.obs_center).sqnorm())
    except AttributeError:
        pass

SimpleVehicle2d.vehiclelog = vehiclelog
######################################################

if __name__ == "__main__":
    # Display set-up
    SCREEN_SIZE = (800,640)
    #screen, bgcolor = pgrender.setup(SCREEN_SIZE, 'Obstacle steering test.')
    # Waypath constants
    UPDATE_SPEED = 0.35
    BORDER = 30
    MIN_DIST = 40.0

    # Vehicle/Obstacle constants
    VEH_RADIUS = 20
    OBS_RADIUS = 90
    EXP_K = .01
    PATHS = True

    # Multiplier for number of test vehicles
    VNMUL = 2

    # Number of vehicles and obstacles
    numveh = 50*VNMUL
    numobs = 1
    total = numveh + numobs

    SPACING = 2/VNMUL

    # Waypoint/path information
    pathlen = 2
    min_dist_sq = MIN_DIST**2
    OBS_POS = Point2d(SCREEN_SIZE[0]//2, SCREEN_SIZE[1]//2)

    X_FINISH = SCREEN_SIZE[0]*.8
    Y_OFF = OBS_POS.y - 100
    waylist = [Point2d(SCREEN_SIZE[0]*.15, Y_OFF),
               Point2d(X_FINISH, Y_OFF)
              ]

    init_pos = waylist[0]
    init_vel = Point2d(5,0)

    # Array of vehicles
    vehicles = [SimpleVehicle2d(init_pos+SPACING*Point2d(0,i), Point2d(0,0), VEH_RADIUS, 1.0, 8.0, 6.0) for i in range(numveh)]

    # Static obstacle
    obslist = [SimpleObstacle2d(OBS_POS, init_vel, OBS_RADIUS)]

    # Set up steering behaviours
    for veh in vehicles:
        veh.navigator.set_steering('OBSTACLESKIM', obslist)
        # Destination(s) on other side of the obstacle
        gwaylist = [pt + SPACING*Point2d(0,vehicles.index(veh)) for pt in waylist]
        # Uncomment next line to give all vehicles the same destination
        gwaylist[1] = waylist[1]+Point2d(0,SPACING*(numveh//2))

        gpath = WaypointPath(gwaylist, False)
        veh.navigator.set_steering('WAYPATHRESUME', gpath, EXP_K)

        veh.vehiclelog_init(OBS_POS)
        veh.active = True

    b_running = True
    num_active = len(vehicles)
    ticks = 0
    ### Main loop ###
    while b_running:

        # Update Vehicles via their Navigators (this includes movement)
        for veh in vehicles:
            veh.vehiclelog()
            if veh.active:
                veh.move(UPDATE_SPEED)
                if veh.pos.x > X_FINISH - VEH_RADIUS:
                    veh.active = False
                    veh.traveltime = ticks*UPDATE_SPEED
                    num_active = num_active - 1

        # Stop once all vehicles have passed their waypoints
        if min([veh.pos.x for veh in vehicles]) > X_FINISH - VEH_RADIUS:
            b_running = False

        ticks = ticks + 1

    ### Post simulation plotting
    from matplotlib import pyplot as plt

    # Vehicle paths
    ax1 = plt.subplot(311)

    for veh in vehicles:
        ax1.plot(veh.xdata, veh.ydata)
    waypt = plt.Circle(OBS_POS.ntuple, OBS_RADIUS+VEH_RADIUS, color='b', fill=False)
    ax1.add_artist(waypt)

    # Maximum collision depth
    from math import sqrt
    EXT_RAD = OBS_RADIUS + VEH_RADIUS
    max_clist = [max(EXT_RAD - sqrt(min(veh.dsqdata)), 0) for veh in vehicles]
    ax2 = plt.subplot(312)
    ax2.plot([x for x in max_clist if x >= 0],'.')
    ax2.set_xlabel('Vehicle number')
    ax2.set_ylabel('Max. Collision Depth')

    ax3 = plt.subplot(313)
    ax3.plot([veh.traveltime for veh in vehicles],'.')
    ax3.set_xlabel('Vehicle number')
    ax3.set_ylabel('Time to destination')

    info = (numveh, VEH_RADIUS, OBS_RADIUS, EXP_K)
    ax1.set_title('Tanget line obstacle avoid\n%d vehicles, VehRad = %s, ObsRad = %s, WayPathReturn k = %s' % info)
    plt.show()

