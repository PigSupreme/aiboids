#!/usr/bin/env python
"""Improved TAKECOVER/STALKING steering demo."""

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
from aiboids.vehicle2d import SimpleVehicle2d
from aiboids import pgrender

if __name__ == "__main__":
    # Display set-up
    SCREEN_SIZE = (1024, 768)
    screen, bgcolor = pgrender.setup(SCREEN_SIZE, 'TAKECOVER/STALKING steering demo.')
    BORDER = 30
    UPDATE_SPEED = 0.5

    # Used to generate random positions and velocities for vehicles
    randpoint = lambda: Point2d(randint(BORDER, SCREEN_SIZE[0]-BORDER), randint(BORDER, SCREEN_SIZE[1]-BORDER))

    # Number of vehicles and obstacles
    numveh = 3
    numobs = 20
    total = numveh + numobs

    # Load images
    images = dict()
    images['green'] = pgrender.boid_chevron(20, (0,222,0), (0,0,0))
    images['yellow'] = pgrender.boid_chevron(20, (222,222,0), (0,0,0))
    images['purple'] = pgrender.boid_chevron(20, (144,33,222), (0,0,0))

    # Randomly generate initial placement for vehicles
    init_pos = [randpoint() for i in range(numveh)]
    init_vel = Point2d(1.0,0)

    # Array of vehicles and associated pygame sprites
    green = SimpleVehicle2d(init_pos[0], init_vel, 20, 1.0, 10.0, 6.0, images['green'])
    yellow = SimpleVehicle2d(init_pos[1], init_vel, 20, 1.0, 7.0, 6.0, images['yellow'])
    purple = SimpleVehicle2d(init_pos[2], init_vel, 20, 1.0, 7.0, 1.0, images['purple'])
    vehicles = [green, yellow, purple]
    rgroup = [veh.sprite for veh in vehicles]

    # Static obstacles for pygame (randomly-generated positions)
    obslist, obs_sprites = pgrender.scattered_obstacles(numobs, 12, SCREEN_SIZE)
    rgroup.extend(obs_sprites)

    # Static Walls for pygame (near screen boundary only)
    wall_list, wall_sprites = pgrender.boundary_walls(SCREEN_SIZE)
    rgroup.extend(wall_sprites)

    # Set-up pygame rendering
    allsprites = pygame.sprite.RenderPlain(rgroup)

    # All vehicles avoid obstacles and walls
    for veh in vehicles:
        veh.navigator.set_steering('OBSTACLEAVOID', obslist)
        veh.navigator.set_steering('WALLAVOID', 30.0, wall_list)

    # Green (WANDER aimlessly, target of TAKECOVER)
    green.navigator.set_steering('WANDER', 200, 30, 20)

    # Yellow (TAKECOVER from Green, rather cowardly)
    yellow.navigator.set_steering('TAKECOVER', green, obslist, 100.0)

    # Purple (TAKECOVER from Green, with stalking)
    purple.navigator.set_steering('TAKECOVER', green, obslist, 500.0, stalk=True)

    ### Main loop ###
    while 1:
        for event in pygame.event.get():
            if event.type in [QUIT, MOUSEBUTTONDOWN]:
                pygame.quit()
                sys.exit()

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
