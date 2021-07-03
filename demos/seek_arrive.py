#!/usr/bin/env python3
"""
seek_arrive.py: SEEK and ARRIVE steering demo.

* Green: Reynolds' SEEK. Notice how this overshoots the target point and creates jitter.
* Yellow: Reynolds' ARRIVE with default hesitance (small).
* Red: Reynolds' ARRIVE with larger hesitance.
"""

import pygame, sys, os
from pygame.locals import QUIT, MOUSEBUTTONDOWN
from random import randint

INF = float('inf')

# Allows running this demo from arbitrary location without installing the package
mydir = os.path.dirname(os.path.realpath(__file__))
mypar = os.path.abspath(os.path.join(mydir, os.pardir))
sys.path.append(mypar)

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
    total = numveh + numtargs

    # Images for pygame sprites
    images = dict()
    images['green'] = pgrender.boid_chevron(20, (0,222,0))
    images['yellow'] = pgrender.boid_chevron(20, (222,222,0))
    images['red'] = pgrender.boid_chevron(25, (222,0,0))

    # Steering behaviour target images (generated here)
    img_targ = pygame.Surface((5,5))
    img_targ.fill((0,0,0))
    images['target'] = img_targ

    # Randomly generate initial placement for vehicles
    init_pos = [randpoint() for i in range(numveh)]
    init_vel = Point2d(1.0,0)

    # Array of vehicles and associated pygame sprites
    green = SimpleVehicle2d(init_pos[0], init_vel, 20, 1.0, 8.0, 6.0, images['green'])
    yellow = SimpleVehicle2d(init_pos[1], init_vel, 20, 1.0, 8.0, 6.0, images['yellow'])
    red = SimpleVehicle2d(init_pos[2], init_vel, 25, 1.0, 8.0, 6.0, images['red'])
    vehicles = [green, yellow, red]
    rgroup = [veh.sprite for veh in vehicles]

    # Steering behaviour targets
    targs = list()
    for i in range(numveh, numveh + numtargs):
        target = BasePointMass2d(randpoint(), ZERO_VECTOR, 10, 0.0, 0.0, 0.0, images['target'])
        targs.append(target)
        rgroup.append(target.sprite)

    # Set-up pygame rendering
    allsprites = pygame.sprite.RenderPlain(rgroup)

    # Green (SEEK demo)
    green.navigator.set_steering('SEEK', targs[0].pos)

    # Yellow (ARRIVE demo)
    yellow.navigator.set_steering('ARRIVE', targs[1].pos)

    # Red (ARRIVE with hesitance)
    red.navigator.set_steering('ARRIVE', targs[2].pos, 8.0)

    b_running = True
    ticks = 0
    TARGET_FREQ = 75
    ### Main loop ###
    while b_running:
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

        # Check for exit
        for event in pygame.event.get():
            if event.type in [QUIT, MOUSEBUTTONDOWN]:
                b_running = False

    ### End of main loop ###
    pygame.quit()
