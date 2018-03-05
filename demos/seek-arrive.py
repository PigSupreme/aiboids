#!/usr/bin/env python
"""New SEEK-ARRIVE steering demo."""

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
from aiboids.point2d import Point2d, ZERO_VECTOR
from aiboids.vehicle2d import BasePointMass2d, SimpleVehicle2d
from aiboids import pgrender


if __name__ == "__main__":
    # Display set-up
    SCREEN_SIZE = (800,640)
    screen, bgcolor = pgrender.setup(SCREEN_SIZE, 'SEEK/ARRIVE steering demo.')
    UPDATE_SPEED = 1.0
    BORDER = 30

    # Used to generate random positions and velocities for vehicles
    randpoint = lambda: Point2d(randint(BORDER, SCREEN_SIZE[0]-BORDER), randint(BORDER, SCREEN_SIZE[1]-BORDER))

    # Number of vehicles and obstacles
    numveh = 3
    numtargs = numveh
    numobs = 0
    total = numveh + numtargs + numobs

    # Load images
    images = dict()
    images['green'] = pgrender.boid_chevron(20, (0,222,0))
    images['yellow'] = pgrender.boid_chevron(20, (222,222,0))
    images['red'] = pgrender.boid_chevron(25, (222,0,0))

    # Steering behaviour target images (generated here)
    img_targ = pygame.Surface((5,5))
    img_targ.fill((0,0,0))
#    img_rect = img_targ.get_rect()
    images['target'] = img_targ

    # Randomly generate initial placement for vehicles
    init_pos = [randpoint() for i in range(numveh)]
    init_vel = Point2d(1.0,0)

    # Array of vehicles and associated pygame sprites
    green = SimpleVehicle2d(init_pos[0], 50, init_vel, images['green'])
    yellow = SimpleVehicle2d(init_pos[1], 50, init_vel, images['yellow'])
    red = SimpleVehicle2d(init_pos[2], 50, init_vel, images['red'])
    vehicles = [green, yellow, red]
    rgroup = [veh.sprite for veh in vehicles]

    # Steering behaviour targets
    targs = list()
    for i in range(numveh, numveh + numtargs):
        target = BasePointMass2d(randpoint(), 10, ZERO_VECTOR, images['target'])
        targs.append(target)
        rgroup.append(target.sprite)

    # Static obstacles for pygame (randomly-generated positions)
    obslist, obs_sprites = pgrender.scattered_obstacles(numobs, 15, SCREEN_SIZE)
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

    # Green (SEEK demo)
    green.navigator.set_steering('SEEK', targs[0].pos)

    # Yellow (ARRIVE demo)
    yellow.navigator.set_steering('ARRIVE', targs[1].pos)

    # Red (ARRIVE with hesitance)
    red.navigator.set_steering('ARRIVE', targs[2].pos, 8.0)

    ### Main loop ###
    ticks = 0
    TARGET_FREQ = 75
    while 1:
        for event in pygame.event.get():
            if event.type in [QUIT, MOUSEBUTTONDOWN]:
                pygame.quit()
                sys.exit()

        # Update Vehicles via their Navigators (this includes movement)
        for veh in vehicles:
            veh.move(UPDATE_SPEED)

        # Update steering targets every so often
        ticks += 1
        if ticks == TARGET_FREQ:
            # Green target
            new_pos = randpoint()
            targs[0].pos = new_pos
            green.navigator.set_steering('SEEK', targs[0].pos)
        if ticks == 2*TARGET_FREQ:
            # Yellow target
            new_pos = randpoint()
            targs[1].pos = new_pos
            yellow.navigator.set_steering('ARRIVE', targs[1].pos)
        if ticks == 3*TARGET_FREQ:
            # Red target
            new_pos = randpoint()
            targs[2].pos = new_pos
            red.navigator.set_steering('ARRIVE', targs[2].pos, 8.0)
            ticks = 0

        # Update Sprites (via pygame sprite group update)
        allsprites.update(UPDATE_SPEED)
        pygame.time.delay(20)

        # Screen update
        screen.fill(bgcolor)
        allsprites.draw(screen)
        pygame.display.flip()

