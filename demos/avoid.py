#!/usr/bin/env python3
"""
avoid.py: OBSTACLEAVOID versus OBSTACLESKIM steering demo.

* Green Boid uses Reynolds-style OBSTACLEAVOID.
* Yellow Boid steers so that its bounding circle is tangent to that of the obstacle.
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
from aiboids import pgrender

if __name__ == "__main__":
    print(os.path.dirname(os.path.realpath(__file__)))
    # Display set-up
    SCREEN_SIZE = (1024, 768)
    screen, bgcolor = pgrender.setup(SCREEN_SIZE, 'Obstacle avoidance/skim steering demo.')
    BORDER = 30
    UPDATE_SPEED = 0.5

    # Used to generate random positions and velocities for vehicles
    randpoint = lambda: Point2d(randint(BORDER, SCREEN_SIZE[0]-BORDER), randint(BORDER, SCREEN_SIZE[1]-BORDER))

    # Number of vehicles and obstacles
    numveh = 5*2
    numobs = 20
    total = numveh + numobs

    # Load images
    VEH_RADIUS = 20
    images = dict()
    images['green'] = pgrender.boid_chevron(VEH_RADIUS, (0,222,0), (0,0,0))
    images['yellow'] = pgrender.boid_chevron(VEH_RADIUS, (222,222,0), (0,0,0))

    # Vehicle (radius, mass, maxspeed, maxforce, spritedata)
    AVOID_DATA = (VEH_RADIUS, 1.0, 8.0, 6.0, images['green'])
    SKIM_DATA = (VEH_RADIUS, 1.0, 8.0, 6.0, images['yellow'])

    # Array of vehicles and associated pygame sprites
    green, yellow = [], []
    for _ in range(numveh//2):
        green.append(SimpleVehicle2d(randpoint(), Point2d(0,0), *AVOID_DATA))
        yellow.append(SimpleVehicle2d(randpoint(), Point2d(0,0), *SKIM_DATA))
    vehicles = green + yellow
    rgroup = [veh.sprite for veh in vehicles]

    # Wall and obstacle data
    WHISKER_FRONT = VEH_RADIUS*1.25
    WHISKER_SIDES = WHISKER_FRONT*1.1
    WALL_WHISKERS = [WHISKER_FRONT*Point2d(1,0),
                     WHISKER_SIDES*Point2d(1,1).unit(),
                     WHISKER_SIDES*Point2d(1,-1).unit()
                    ]
    OBS_RADIUS = 12

    # Static obstacles for pygame (randomly-generated positions)
    obslist, obs_sprites = pgrender.scattered_obstacles(numobs, OBS_RADIUS, SCREEN_SIZE)
    rgroup.extend(obs_sprites)

    # Static Walls for pygame (near screen boundary only)
    wall_list, wall_sprites = pgrender.boundary_walls(SCREEN_SIZE)
    rgroup.extend(wall_sprites)

    # Set-up pygame rendering
    allsprites = pygame.sprite.RenderPlain(rgroup)

    # All vehicles wander and avoid walls
    for veh in vehicles:
        veh.navigator.set_steering('WANDER', 200, 30, 20)
        veh.navigator.set_steering('WALLAVOID', WALL_WHISKERS, wall_list)

    # Green uses traditional OBSTACLEVAOID
    for veh in green:
        veh.navigator.set_steering('OBSTACLEAVOID', obslist)

    # Yellow uses new OBSTACLESKIM
    for veh in yellow:
        veh.navigator.set_steering('OBSTACLEAVOID', obslist)

    b_running = True
    ### Main loop ###
    while b_running:
        # Update Vehicles via their Navigators (this includes movement)
        for veh in vehicles:
            veh.move(UPDATE_SPEED)

        # Update Sprites (via pygame sprite group update)
        allsprites.update(UPDATE_SPEED)
        pygame.time.delay(10)

        # Screen update
        screen.fill(bgcolor)
        allsprites.draw(screen)
        pygame.display.flip()

        # Check for exit
        for event in pygame.event.get():
            if event.type in [QUIT, MOUSEBUTTONDOWN]:
                b_running = False

        ## End of main loop ##

    pygame.quit()
