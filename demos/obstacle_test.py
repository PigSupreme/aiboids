#!/usr/bin/env python
"""Improved Waypoint steering demo."""

# for python3 compat
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import sys, pygame
from pygame.locals import QUIT, MOUSEBUTTONDOWN
from random import randint

INF = float('inf')

# Note: Adjust this depending on where this file ends up.
sys.path.append('..')
from aiboids.point2d import Point2d
from aiboids.vehicle2d import SimpleVehicle2d, SimpleObstacle2d
from aiboids.steering import WaypointPath

from aiboids import pgrender

if __name__ == "__main__":
    # Display set-up
    SCREEN_SIZE = (800,640)
    screen, bgcolor = pgrender.setup(SCREEN_SIZE, 'Obstacle steering test.')
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

    # Array of vehicles and associated pygame sprites
    image = pgrender.boid_chevron(VEH_RADIUS, (0,222,0), (0,0,0))
    vehicles = [SimpleVehicle2d(init_pos+SPACING*Point2d(0,i), VEH_RADIUS, init_vel, image) for i in range(numveh)]
    rgroup = [veh.sprite for veh in vehicles]

    # Static obstacle
    obs_img = pgrender.obstacle_bumper(OBS_RADIUS)
    obslist = [SimpleObstacle2d(OBS_POS, OBS_RADIUS, obs_img)]
    rgroup.extend([obs.sprite for obs in obslist])

    # Set-up pygame rendering
    allsprites = pygame.sprite.RenderPlain(rgroup)

    # All vehicles avoid obstacles and walls
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

        # Update Sprites (via pygame sprite group update)
        allsprites.update(UPDATE_SPEED)
        pygame.time.delay(20)

        # Screen update
        screen.fill(bgcolor)
        #pygame.draw.lines(screen, (55,55,55), True, waylist, 2)
        allsprites.draw(screen)
        pygame.display.flip()

        # Stop once all vehicles have passed their waypoints
        if min([veh.pos.x for veh in vehicles]) > X_FINISH - VEH_RADIUS:
            b_running = False

    ### Post simulation plotting
    pygame.quit()
    from matplotlib import pyplot
    ax = pyplot.axes()
    #pyplot.plot(green.ydata)
    for veh in vehicles:
        ax.plot(veh.xdata, veh.ydata)
    waypt = pyplot.Circle(OBS_POS.ntuple, OBS_RADIUS+VEH_RADIUS, color='b', fill=False)
    ax.add_artist(waypt)
