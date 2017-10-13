#!/usr/bin/env python
"""Improved TAKECOVER/STALKING steering demo."""

# for python3 compat
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import sys, pygame
from pygame.locals import QUIT, MOUSEBUTTONDOWN
from random import randint, shuffle

INF = float('inf')

# Note: Adjust this depending on where this file ends up.
sys.path.append('..')
from aiboids.point2d import Point2d
from aiboids.vehicle2d import load_pygame_image
from aiboids.vehicle2d import BasePointMass2d, BaseWall2d, SimpleObstacle2d

ZERO_VECTOR = Point2d(0,0)

import aiboids.steering as steering

if __name__ == "__main__":
    pygame.init()

    # Display constants
    size = sc_width, sc_height = 800, 640
    screen = pygame.display.set_mode(size)
    pygame.display.set_caption('TAKECOVER/STALKING steering demo.')
    bgcolor = (111, 145, 192)
    randpos = lambda: (randint(30, sc_width-30), randint(30, sc_height-30))

    # Update Speed
    UPDATE_SPEED = 0.2

    # Number of vehicles and obstacles
    numveh = 2
    numtargs = 0
    numobs = 8
    total = numveh + numtargs + numobs

    # Load images
    images = dict()
    images['green'] = load_pygame_image('../images/gpig.png', -1)
    images['yellow'] = load_pygame_image('../images/ypig.png', -1)
    #images['red'] = load_pygame_image('../images/rpig.png',-1)
    images['obstacle'] = load_pygame_image('../images/circle.png', -1)

    # Randomly generate initial placement for vehicles
    init_pos = [Point2d(*randpos()) for i in range(numveh)]
    init_vel = Point2d(1.0,0)

    # Array of vehicles and associated pygame sprites
    green = BasePointMass2d(init_pos[0], 50, init_vel, images['green'])
    yellow = BasePointMass2d(init_pos[1], 50, init_vel, images['yellow'])
    #red = BasePointMass2d(init_pos[2], 50, init_vel, images['red'])
    vehicles = [green, yellow]#, red]
    rgroup = [veh.sprite for veh in vehicles]

    # Static obstacles for pygame (randomly-generated positions)
    # Don't ask how this works; it avoids clustering the obstacles
    yoffset = sc_height//(numobs+1)
    yvals = list(range(yoffset, sc_height-yoffset, yoffset))
    shuffle(yvals)
    obslist = list()
    for i in range(2*numveh, 2*numveh + numobs):
        offset = (i+1.0-2*numveh)/(numobs+1)
        rany = yvals[i-2*numveh]
        new_pos = Point2d(offset*sc_width, rany)
        obstacle = SimpleObstacle2d(new_pos, 10, images['obstacle'])
        obslist.append(obstacle)
        rgroup.append(obstacle.sprite)

    # Static Walls for pygame (screen border only)
    wall_list = (BaseWall2d((sc_width//2, 10), sc_width-20, 4, Point2d(0,1)),
                 BaseWall2d((sc_width//2, sc_height-10), sc_width-20, 4, Point2d(0,-1)),
                 BaseWall2d((10, sc_height//2), sc_height-20, 4, Point2d(1,0)),
                 BaseWall2d((sc_width-10,sc_height//2), sc_height-20, 4, Point2d(-1,0)))
    wall_list = []
    for wall in wall_list:
        rgroup.append(wall.sprite)

    # Set-up pygame rendering
    allsprites = pygame.sprite.RenderPlain(rgroup)

    # All vehicles avoid obstacles and walls
    for veh in vehicles:
        steering.Navigator(veh)
        veh.navigator.set_steering('OBSTACLEAVOID', obslist)
        veh.navigator.set_steering('WALLAVOID', 30.0, wall_list)

    # Green (SEEK demo)
    green.navigator.set_steering('ARRIVE', 0.5*Point2d(*size))

    # Yellow (ARRIVE demo)
    yellow.navigator.set_steering('TAKECOVER', green, obslist, 250, True)

    # Red (ARRIVE with hesitance)
    #red.navigator.set_steering('ARRIVE', targs[2].pos, 8.0)

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
            veh.navigator.update()

        # Update steering targets every so often
        ticks += 1
        if ticks == TARGET_FREQ:
            # Green tracks yellow
            new_pos = Point2d(*randpos())
            green.navigator.set_steering('ARRIVE', yellow.pos + 50*yellow.front)
        if ticks == 3*TARGET_FREQ:
            green.navigator.set_steering('ARRIVE', yellow.pos - 50*yellow.front)
            ticks = 0

        # Update Sprites (via pygame sprite group update)
        allsprites.update(UPDATE_SPEED)

        pygame.time.delay(20)

        # Screen update
        screen.fill(bgcolor)
        allsprites.draw(screen)
        pygame.display.flip()

    pygame.time.delay(2000)