#!/usr/bin/env python3
"""Waypath with obstacle interaction test. Matplotlib only."""

import sys
try:
    from matplotlib import pyplot as plt
except ImportError:
    print("Please install matplotlib to run this demo.")
    sys.exit()
INF = float('inf')

# Note: Adjust this depending on where this file ends up.
sys.path.append('..')
from aiboids.point2d import Point2d
from aiboids.vehicle2d import SimpleVehicle2d, SimpleObstacle2d
from aiboids.steering import WaypointPath

if __name__ == "__main__":
    # "Display" set-up
    SCREEN_SIZE = (800,640)
    UPDATE_SPEED = 1.0
    BORDER = 30
    MIN_DIST = 40.0

    VEH_RADIUS = 20
    OBS_RADIUS = 60

    ######################################################
    # Duck punching!
    def vehicledata(veh):
        return [veh.pos.x, veh.pos.y, veh.vel.x, veh.vel.y]

    def vehiclelog_init(veh):
        veh.xdata = list()
        veh.ydata = list()
        veh.vxdata = list()
        veh.vydata = list()
    SimpleVehicle2d.vehiclelog_init = vehiclelog_init

    def vehiclelog(veh):
        veh.xdata.append(veh.pos.x)
        veh.ydata.append(SCREEN_SIZE[1] - veh.pos.y)
        veh.vxdata.append(veh.vel.x)
        veh.vydata.append(-veh.vel.y)
    SimpleVehicle2d.vehiclelog = vehiclelog
    ######################################################

    # Number of vehicles and obstacles
    numveh = 20
    numobs = 1
    total = numveh + numobs
    SPACING = 10

    # Waypoint/path and obstacle information
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
    vehicles = [SimpleVehicle2d(init_pos+SPACING*Point2d(0,i), init_vel, VEH_RADIUS, 1.0, 8.0, 6.0) for i in range(numveh)]

    # Static obstacle
    obslist = [SimpleObstacle2d(OBS_POS, Point2d(0,0), OBS_RADIUS)]

    # Steering behaviours
    for veh in vehicles:
        veh.navigator.set_steering('OBSTACLEAVOID', obslist)

        # Destination(s) on other side of the obstacle
        gwaylist = [pt + SPACING*Point2d(0,vehicles.index(veh)) for pt in waylist]
        # Uncomment next line to give all vehicles the same destination
        #gwaylist[1] = waylist[1]+Point2d(0,SPACING*(numveh//2))

        gpath = WaypointPath(gwaylist, False)
        veh.navigator.set_steering('WAYPATHRESUME', gpath, .005)
        veh.vehiclelog_init()

    b_running = True
    ### Main loop ###
    while b_running:

        # Update Vehicles via their Navigators (this includes movement)
        for veh in vehicles:
            veh.move(UPDATE_SPEED)
            # Logging info
            veh.vehiclelog()

        # Stop once all vehicles have passed their waypoints
        if min([veh.pos.x for veh in vehicles]) > X_FINISH - VEH_RADIUS:
            b_running = False

    ### Post simulation plotting
    ax = plt.axes()
    for veh in vehicles:
        ax.plot(veh.xdata, veh.ydata)
    waypt = plt.Circle(OBS_POS.ntuple, OBS_RADIUS+VEH_RADIUS, color='b', fill=False)
    ax.add_artist(waypt)
    plt.show()
