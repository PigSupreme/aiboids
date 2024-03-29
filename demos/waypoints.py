#!/usr/bin/env python3
"""
waypoints.py: Improved Waypoint steering demo.

Green: WAYPATHVISIT will SEEK directly towards each waypoint in turn, traversing
the path once and ARRIVING at last waypoint (and stopping).

Yellow: WAYPATHRESUME attempts to "stay on the path" between the last and next
waypoint as much as possible. If an obstacle forces the boid off of the path, it
will not directly SEEK the next waypoint, but stike a balance between returning
to the path and making foward progress. Uses a cyclic path, repeating forever.

Obstacles ensure that the boids will need to divert from the shortest path. All
waypoints and obstacles are random.
"""

import pygame, sys, os
from pygame.locals import QUIT, MOUSEBUTTONDOWN
from random import randint

INF = float('inf')

# Allows running this demo from arbitrary location without installing the package
mydir = os.path.dirname(os.path.realpath(__file__))
mypar = os.path.abspath(os.path.join(mydir, os.pardir))
sys.path.append(mypar)

from aiboids.point2d import Point2d
from aiboids.vehicle2d import SimpleVehicle2d
from aiboids.steering import WaypointPath

from aiboids import pgrender


if __name__ == "__main__":
    # Display set-up
    SCREEN_SIZE = (800,640)
    screen, bgcolor = pgrender.setup(SCREEN_SIZE, 'Waypoints steering demo.')
    UPDATE_SPEED = 1.0
    BORDER = 30

    # Used to generate random positions and velocities for vehicles
    randpoint = lambda: Point2d(randint(BORDER, SCREEN_SIZE[0]-BORDER), randint(BORDER, SCREEN_SIZE[1]-BORDER))

    # Number of vehicles and obstacles
    numveh = 2
    numobs = 8
    total = numveh + numobs

    # Waypoint/path information
    pathlen = 6
    min_dist_sq = 80**2

    # Load images
    images = dict()
    images['green'] = pgrender.boid_chevron(20, (0,222,0), (0,0,0))
    images['yellow'] = pgrender.boid_chevron(20, (222,222,0), (0,0,0))

    # Randomly generate initial placement for vehicles
    init_pos = [randpoint() for i in range(numveh)]
    init_vel = Point2d(1.0,0)

    # Array of vehicles and associated pygame sprites
    green = SimpleVehicle2d(init_pos[0], init_vel, 20, 1.0, 8.0, 6.0, images['green'])
    yellow = SimpleVehicle2d(init_pos[1], init_vel, 20, 1.0, 8.0, 6.0, images['yellow'])
    vehicles = [green, yellow]
    rgroup = [veh.sprite for veh in vehicles]

    # Static obstacles for pygame (randomly-generated positions)
    obslist, obs_sprites = pgrender.scattered_obstacles(numobs, 10, SCREEN_SIZE)
    rgroup.extend(obs_sprites)

    # Set-up pygame rendering
    allsprites = pygame.sprite.RenderPlain(rgroup)

    # All vehicles avoid obstacles
    for veh in vehicles:
        veh.navigator.set_steering('OBSTACLEAVOID', obslist)

    # Randomly-generated list of waypoints for all vehicles, not too close
    # to any obstacles
    waylist = []
    while len(waylist) <= pathlen:
        newp = randpoint()
        d_min_sq = min([(obs.pos - newp).sqnorm() for obs in obslist])
        if d_min_sq > min_dist_sq:
            waylist.append(newp)

    # Green (WAYPATHVISIT; stops at final waypoint)
    glist = waylist
    gpath = WaypointPath(2*[Point2d(*p) for p in glist], False)
    gpath.resume_at_nearest_from(green.pos)
    green.navigator.set_steering('WAYPATHVISIT', gpath)

    # Yellow (WAYPATHVISIT with cyclic path)
    ylist = waylist[::-1] + waylist[-1:]
    ypath = WaypointPath([Point2d(*p) for p in ylist], True)
    ypath.resume_at_nearest_from(yellow.pos)
    yellow.navigator.set_steering('WAYPATHRESUME', ypath)

    b_running = True
    ### Main loop ###
    while b_running:
        # Update Vehicles via their Navigators (this includes movement)
        for veh in vehicles:
            veh.move(UPDATE_SPEED)

        # Update Sprites (via pygame sprite group update)
        allsprites.update(UPDATE_SPEED)
        pygame.time.delay(20)

        # Screen update
        screen.fill(bgcolor)
        pygame.draw.lines(screen, (55,55,55), True, waylist, 2)
        allsprites.draw(screen)
        pygame.display.flip()

        # Check for exit
        for event in pygame.event.get():
            if event.type in [QUIT, MOUSEBUTTONDOWN]:
                b_running = False

    ### End of main loop ###
    pygame.quit()
