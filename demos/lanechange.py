#!/usr/bin/env python3
"""
lanechange.py: Corridor/lane traversal using SIDESLIP.

The boid uses SIDESLIP (essentially logistic growth between parallel paths) to control lane change.
Walls are visual aids only; WALLAVOID isn't used here.
"""

import pygame, sys, os
from pygame.locals import QUIT, MOUSEBUTTONDOWN

INF = float('inf')
SQRT2 = 1.4142135623730951

# Allows running this demo from arbitrary location without installing the package
mydir = os.path.dirname(os.path.realpath(__file__))
mypar = os.path.abspath(os.path.join(mydir, os.pardir))
sys.path.append(mypar)

from aiboids.point2d import Point2d
from aiboids.vehicle2d import SimpleVehicle2d, BaseWall2d

from aiboids import pgrender

if __name__ == "__main__":
    # Display set-up
    SCREEN_SIZE = (960, 800)
    screen, bgcolor = pgrender.setup(SCREEN_SIZE, 'Corridor/lane SIDESLIP demo.')
    UPDATE_SPEED = 0.5

    # Sideslip information
    WALL_THICKNESS = 5
    OFFSETX, OFFSETY = 50, 15
    RETURNX = 500
    DRIFT = 2*OFFSETY + WALL_THICKNESS
    DRIFTSLOPE = .5

    corlen = SCREEN_SIZE[0] - 2*OFFSETX
    min_dist_sq = 80**2
    midx = SCREEN_SIZE[0]//2
    midy = SCREEN_SIZE[1]//2

    # Initial placement for vehicle
    VEH_RADIUS = 16
    LEAD_OFFSET = 100
    INIT_SPEED = 12.0

    base_pos = Point2d(OFFSETX, midy)
    base_dir = Point2d(1.0,0.0).unit()
    init_pos = base_pos - LEAD_OFFSET*base_dir
    init_vel = INIT_SPEED*base_dir

    # Vehicle and pygame sprite
    green_image = pgrender.boid_chevron(VEH_RADIUS, (0,222,0), (0,0,0))
    green = SimpleVehicle2d(init_pos, init_vel, VEH_RADIUS, 1.0, INIT_SPEED, 6.0, green_image)
    vehicles = [green]
    rgroup = [veh.sprite for veh in vehicles]

    # Static Walls (used for corridor/lane visuals only)
    spdata = pgrender.WALL_DATA
    wall_list = [BaseWall2d((midx, midy-OFFSETY), corlen, WALL_THICKNESS, Point2d(0,1), spdata),
                 BaseWall2d((midx, midy+OFFSETY), corlen, WALL_THICKNESS, Point2d(0,-1), spdata),
                 BaseWall2d((midx, midy+3*OFFSETY+WALL_THICKNESS), corlen, WALL_THICKNESS, Point2d(0,-1), spdata)
                ]
    rgroup.extend([wall.sprite for wall in wall_list])

    # Set-up pygame rendering
    allsprites = pygame.sprite.RenderPlain(rgroup)

    # Green Steering
    nav = green.navigator
    nav.set_steering('SIDESLIP', base_dir, DRIFT, DRIFTSLOPE)
    nav.pause_steering('SIDESLIP')

    in_lane = False
    out_lane = False
    b_running = True

    ### Main loop ###
    while b_running:

        # Update Vehicles via their Navigators (this includes movement)
        for veh in vehicles:
            veh.move(UPDATE_SPEED)

        # Once we're in the lane, activate SIDESLIP
        if not in_lane and green.pos.x > OFFSETX:
            nav.resume_steering('SIDESLIP')
            in_lane = True

        # Return to the original lane
        elif not out_lane and green.pos.x > RETURNX:
            nav.set_steering('SIDESLIP', base_dir, -DRIFT, DRIFTSLOPE)
            out_lane = True

        # Warp to start and repeat
        if green.pos.x > SCREEN_SIZE[0]:
            green.pos = init_pos
            nav.set_steering('SIDESLIP', base_dir, DRIFT, DRIFTSLOPE)
            nav.pause_steering('SIDESLIP')
            in_lane = False
            out_lane = False

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
