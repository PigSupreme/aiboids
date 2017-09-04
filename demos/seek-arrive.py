#!/usr/bin/env python
"""New SEEK-ARRIVE steering demo."""

# for python3 compat
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import sys, pygame
from pygame.locals import QUIT, MOUSEBUTTONDOWN
from random import randint, shuffle

TARGET_FREQ = 500

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
    pygame.display.set_caption('SEEK-ARRIVE Steering Demo')
    bgcolor = (111, 145, 192)
    randpos = lambda: (randint(30, sc_width-30), randint(30, sc_height-30))

    # Update Speed
    UPDATE_SPEED = 0.2

    # Number of vehicles and obstacles
    numveh = 3
    numtargs = numveh
    numobs = 0
    total = numveh + numtargs + numobs

    # Load images
    images = dict()
    images['green'] = load_pygame_image('../images/gpig.png', -1)
    images['yellow'] = load_pygame_image('../images/ypig.png', -1)
    images['red'] = load_pygame_image('../images/rpig.png',-1)
    images['obstacle'] = load_pygame_image('../images/circle.png', -1)

    # Steering behaviour target images (generated here)
    img_targ = pygame.Surface((5,5))
    img_targ.fill((0,0,0))
    img_rect = img_targ.get_rect()
    images['target'] = (img_targ, img_rect)

    # Randomly generate initial placement for vehicles
    init_pos = [Point2d(*randpos()) for i in range(numveh)]
    init_vel = Point2d(1.0,0)

    # Array of vehicles and associated pygame sprites
    green = BasePointMass2d(init_pos[0], 50, init_vel, images['green'])
    yellow = BasePointMass2d(init_pos[1], 50, init_vel, images['yellow'])
    red = BasePointMass2d(init_pos[2], 50, init_vel, images['red'])
    vehicles = [green, yellow, red]
    rgroup = [veh.sprite for veh in vehicles]

    # Steering behaviour targets
    targs = list()
    for i in range(numveh, numveh + numtargs):
        target = BasePointMass2d(Point2d(*randpos()), 10, ZERO_VECTOR, images['target'])
        targs.append(target)
        rgroup.append(target.sprite)

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
    for wall in wall_list:
        rgroup.append(wall.sprite)

    # Set-up pygame rendering
    allsprites = pygame.sprite.RenderPlain(rgroup)

    # Green (SEEK demo)
    green_nav = steering.Navigator(green)
    newseek = steering.Seek(green, targs[0].pos)
    green_nav.active_behaviours= [newseek]
    newavoidg = steering.ObstacleAvoid(green, obslist)
    green_nav.active_behaviours.append(newavoidg)
    wallavoidg = steering.WallAvoid(green, 30.0, wall_list)
    green_nav.active_behaviours.append(wallavoidg)

    # Yellow (ARRIVE demo)
    yellow_nav = steering.Navigator(yellow)
    demo_behy = steering.Arrive(yellow, targs[1].pos)
    yellow_nav.active_behaviours.append(demo_behy)
    newavoidy = steering.ObstacleAvoid(yellow, obslist)
    yellow_nav.active_behaviours.append(newavoidy)
    wallavoidy = steering.WallAvoid(yellow, 30.0, wall_list)
    yellow_nav.active_behaviours.append(wallavoidy)

    # Yellow (ARRIVE demo)
    red_nav = steering.Navigator(red)
    demo_behr = steering.Arrive(red, targs[2].pos, 8.0)
    red_nav.active_behaviours.append(demo_behr)
    newavoidr = steering.ObstacleAvoid(red, obslist)
    red_nav.active_behaviours.append(newavoidr)
    wallavoidr = steering.WallAvoid(red, 40.0, wall_list)
    red_nav.active_behaviours.append(wallavoidr)


    ### Main loop ###
    ticks = 0
    TARGET_FREQ = 75
    while 1:
        for event in pygame.event.get():
            if event.type in [QUIT, MOUSEBUTTONDOWN]:
                pygame.quit()
                sys.exit()

        # Update Vehicles via their Navigator objects
        green_nav.update()
        yellow_nav.update()
        red_nav.update()

        # Update steering targets every so often
        ticks += 1
        if ticks == TARGET_FREQ:
            # Green target
            new_pos = Point2d(*randpos())
            targs[0].pos = new_pos
            del green_nav.active_behaviours[0]
            newseek = steering.Seek(green, new_pos)
            green_nav.active_behaviours.insert(0, newseek)
        if ticks == 2*TARGET_FREQ:
            # Yellow target
            new_pos = Point2d(*randpos())
            targs[1].pos = new_pos
            del yellow_nav.active_behaviours[0]
            newseek = steering.Arrive(yellow, new_pos)
            yellow_nav.active_behaviours.insert(0, newseek)
        if ticks == 3*TARGET_FREQ:
            # Red target
            new_pos = Point2d(*randpos())
            targs[2].pos = new_pos
            del red_nav.active_behaviours[0]
            newseek = steering.Arrive(red, new_pos, 8.0)
            red_nav.active_behaviours.insert(0, newseek)
            ticks = 0

        # Update Sprites (via pygame sprite group update)
        allsprites.update(UPDATE_SPEED)

        pygame.time.delay(20)

        # Screen update
        screen.fill(bgcolor)
        allsprites.draw(screen)
        pygame.display.flip()

    pygame.time.delay(2000)
