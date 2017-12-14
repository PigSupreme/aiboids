#!/usr/bin/env python
"""Flocking demo."""

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
    pygame.display.set_caption('FLOCKING/EVADE steering demo.')
    bgcolor = (111, 145, 192)
    randpos = lambda: (randint(30, sc_width-30), randint(30, sc_height-30))

    # Update Speed
    UPDATE_SPEED = 0.2

    # Number of vehicles and obstacles
    numsheep = 10
    numveh = numsheep + 1
    numobs = 8
    total = numveh + numobs

    # Load images
    images = dict()
    images['green'] = load_pygame_image('../images/gpig.png', -1)
    images['yellow'] = load_pygame_image('../images/ypig.png', -1)
    images['obstacle'] = load_pygame_image('../images/circle.png', -1)

    # Randomly generate initial placement for vehicles
    init_pos = [Point2d(*randpos()) for i in range(numveh)]
    init_vel = [Point2d(*randpos()).unit() for i in range(numveh)]

    # Flock of sheep and associated pygame sprites
    sheep_list = []
    for i in range(numsheep):
         sheep = BasePointMass2d(init_pos[i], 40, init_vel[i], images['green'])
         sheep_list.append(sheep)
    # ...and your little dog, too!
    dog = BasePointMass2d(init_pos[numsheep], 40, init_vel[numsheep], images['yellow'])

    # List of vehicles and sprites for later use
    vehicles = sheep_list + [dog]
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
    #wall_list
    for wall in wall_list:
        rgroup.append(wall.sprite)

    # Set-up pygame rendering
    allsprites = pygame.sprite.RenderPlain(rgroup)


##############################
# Navigator set-up starts here
##############################

    # This demo fails to celebrate its sheep diversity
    # Flock with other sheep and evade the dog
    for sheep in sheep_list:
         nav = steering.Navigator(sheep)
         sheep.flockmates = sheep_list
         nav.set_steering('FLOCKSEPARATE')
         nav.set_steering('FLOCKALIGN')
         nav.set_steering('FLOCKCOHESION')
         nav.set_steering('EVADE', dog, 180)
         nav.set_steering('WANDER', 250, 10, 3)

    # No rule says a dog can't override default physics!
    dog.maxspeed, dog.radius = 10.0, 50
    dog.flockmates = sheep_list
    nav = steering.Navigator(dog)
    nav.set_steering('FLOCKSEPARATE')
    nav.set_steering('FLOCKALIGN')
    nav.set_steering('WANDER', 200, 25, 6)

    # All vehicles avoid obstacles and walls
    for veh in vehicles:
        veh.navigator.set_steering('OBSTACLEAVOID', obslist)
        veh.navigator.set_steering('WALLAVOID', 25.0, wall_list)

##############################
# Main loop
##############################
    ticks = 0
    TARGET_FREQ = 75
    while 1:
        for event in pygame.event.get():
            if event.type in [QUIT, MOUSEBUTTONDOWN]:
                pygame.quit()
                sys.exit()

        # Update Vehicles via their Navigators (this includes movement)
        for veh in vehicles:
            veh.navigator.update(UPDATE_SPEED)

        # Update Sprites (via pygame sprite group update)
        allsprites.update(UPDATE_SPEED)

        pygame.time.delay(2)

        # Screen update
        screen.fill(bgcolor)
        allsprites.draw(screen)
        pygame.display.flip()

    pygame.time.delay(2000)
