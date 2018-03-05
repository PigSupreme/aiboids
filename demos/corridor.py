#!/usr/bin/env python
"""Corridor/lane traversal using WALLAVOID, with end plot."""

# for python3 compat
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import sys
import pygame
from pygame.locals import QUIT, MOUSEBUTTONDOWN

INF = float('inf')

# Note: Adjust this depending on where this file ends up.
sys.path.append('..')
from aiboids.point2d import Point2d
from aiboids.vehicle2d import SimpleVehicle2d, BaseWall2d

from aiboids import pgrender


if __name__ == "__main__":
    # Display set-up
    SCREEN_SIZE = (800, 640)
    screen, bgcolor = pgrender.setup(SCREEN_SIZE, 'Corridor/lane steering demo.')
    UPDATE_SPEED = 0.5

    # Number of vehicles and obstacles
    numveh = 1
    numobs = 0
    total = numveh + numobs

    # Waypoint/path information
    OFFSETX, OFFSETY = 80, 15
    corlen = SCREEN_SIZE[0] - 2*OFFSETX
    min_dist_sq = 80**2
    midx = SCREEN_SIZE[0]//2
    midy = SCREEN_SIZE[1]//2

    # Initial placement for vehicles
    LEAD_OFFSET = 100
    INIT_SPEED = 12
    WHISKER_FRONT = 25.0
    base_pos = Point2d(OFFSETX, midy)
    base_dir = Point2d(1.0,0.5).unit()
    init_pos = base_pos - LEAD_OFFSET*base_dir
    init_vel = INIT_SPEED*base_dir

    # Vehicle and pygame sprite
    green_image = pgrender.boid_chevron(20, (0,222,0), (0,0,0))
    green = SimpleVehicle2d(init_pos, 20, init_vel, green_image)
    green.maxspeed = INIT_SPEED
    vehicles = [green]
    rgroup = [veh.sprite for veh in vehicles]

    # Static Walls (used for corridor/lane)
    spdata = pgrender.WALL_DATA
    wall_list = [BaseWall2d((midx, midy-OFFSETY), corlen, 5, Point2d(0,1), spdata),
                 BaseWall2d((midx, midy+OFFSETY), corlen, 5, Point2d(0,-1), spdata)
                ]
    rgroup.extend([wall.sprite for wall in wall_list])

    # Set-up pygame rendering
    allsprites = pygame.sprite.RenderPlain(rgroup)

    # Green Steering
    nav = green.navigator
    nav.set_steering('ARRIVE', Point2d(SCREEN_SIZE[0], midy), 1.0)
    nav.pause_steering('ARRIVE')
    nav.set_steering('WALLAVOID', WHISKER_FRONT, wall_list)
    nav.pause_steering('WALLAVOID')
    in_lane = False

    # Logging for end plot
    glog = []
    b_running = True

    ### Main loop ###
    while b_running:

        # Update Vehicles via their Navigators (this includes movement)
        glog.append(green.pos.ntuple)
        for veh in vehicles:
            veh.move(UPDATE_SPEED)

        # Once we're in the corridor, activate ARRIVE
        if not in_lane and green.pos.x > OFFSETX:
            nav.resume_steering('ARRIVE')
            nav.resume_steering('WALLAVOID')
            in_lane = True

        # Update Sprites (via pygame sprite group update)
        allsprites.update(UPDATE_SPEED)
        pygame.time.delay(20)

        # Screen update
        screen.fill(bgcolor)
        allsprites.draw(screen)
        pygame.display.flip()

        for event in pygame.event.get():
            if event.type in [QUIT, MOUSEBUTTONDOWN]:
                pygame.quit()
                b_running = False

    from matplotlib.pyplot import plot
    plot([p[0] for p in glog], [p[1] for p in glog])
