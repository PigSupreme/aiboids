#!/usr/bin/env python
"""New avant-garde steering behaviour."""

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
from point2d import Point2d
from vehicle2d import load_pygame_image
# TODO: Change this to an actual vehicle
from vehicle2d import BasePointMass2d
from vehicle2d import BaseWall2d, SimpleObstacle2d

ZERO_VECTOR = Point2d(0,0)

from steering import Navigator, Seek, Flee, Arrive, ObstacleAvoid, WallAvoid

if __name__ == "__main__":
    pygame.init()

    # Display constants
    size = sc_width, sc_height = 800, 640
    screen = pygame.display.set_mode(size)
    pygame.display.set_caption('SEEK - ARRIVE demo')
    bgcolor = 111, 145, 192

    # Update Speed
    UPDATE_SPEED = 0.2

    # Number of vehicles and obstacles
    numveh = 1
    numtargs = numveh
    numobs = 16
    total = numveh + numtargs + numobs

    # Space for Sprite images and pygame rectangles
    img = list(range(total))
    rec = list(range(total))

    # Load vehicle images
    img[0], rec[0] = load_pygame_image('../images/gpig.png', -1)

    # Steering behaviour target images (generated here)
    for i in range(numveh, 2*numveh):
        img[i] = pygame.Surface((5,5))
        rec[i] = img[i].get_rect()

    # Static obstacle image (shared among all obstacles)
    obs_img, obs_rec = load_pygame_image('../images/circle.png', -1)
    for i in range(2*numveh, 2*numveh + numobs):
        img[i], rec[i] = obs_img, obs_rec

    # Randomly generate initial placement for vehicles
    pos = [Point2d(randint(30, sc_width-30), randint(30, sc_height-30)) for i in range(numveh)]
    pos[0] = Point2d(sc_width/2, sc_height/2)
    vel = Point2d(1.0,0)

    # Array of vehicles and associated pygame sprites
    green = BasePointMass2d(pos[0], 50, vel, (img[0], rec[0]))
    vehicles = [green]
    obj = [green]
    rgroup = [green.sprite]

    # Steering behaviour targets (implemented as vehicles for later use...)
    for i in range(numveh, 2*numveh):
        x_new = randint(30, sc_width-30)
        y_new = randint(30, sc_height-30)
        new_pos = Point2d(x_new,y_new)
        target = BasePointMass2d(new_pos, 10, ZERO_VECTOR, (img[i], rec[i]))
        obj.append(target)
        rgroup.append(target.sprite)

    # Static obstacles for pygame (randomly-generated positions)
    yoffset = sc_height//(numobs+1)
    yvals = list(range(yoffset, sc_height-yoffset, yoffset))
    shuffle(yvals)
    for i in range(2*numveh, 2*numveh + numobs):
        offset = (i+1.0-2*numveh)/(numobs+1)
        rany = yvals[i-2*numveh]
        new_pos = Point2d(offset*sc_width, rany)
        obstacle = SimpleObstacle2d(new_pos, 10, (img[i], rec[i]))
        obj.append(obstacle)
        rgroup.append(obstacle.sprite)
    # This gives a convenient list of (non-wall) obstacles for later use
    obslist = obj[2*numveh:]

    # Static Walls for pygame (screen border only)
    wall_list = (BaseWall2d((sc_width//2, 10), sc_width-20, 4, Point2d(0,1)),
                 BaseWall2d((sc_width//2, sc_height-10), sc_width-20, 4, Point2d(0,-1)),
                 BaseWall2d((10, sc_height//2), sc_height-20, 4, Point2d(1,0)),
                 BaseWall2d((sc_width-10,sc_height//2), sc_height-20, 4, Point2d(-1,0)))
    obj.extend(wall_list)
    for wall in wall_list:
        rgroup.append(wall.sprite)

    # Set-up pygame rendering
    allsprites = pygame.sprite.RenderPlain(rgroup)

    ### Vehicle steering behavior defined below ###
    # All vehicles will avoid obstacles and walls
#    for i in range(3):
#        obj[i].steering.set_target(AVOID=obslist, WALLAVOID=[30, wall_list])
    ### End of vehicle behavior ###

    # Green (SEEK)
    green_nav = Navigator(green)
    newseek = Seek(green, obj[1].pos)
    green_nav.active_behaviours.append(newseek)
    newavoid = ObstacleAvoid(green, obslist)
    green_nav.active_behaviours.append(newavoid)
    wallavoid = WallAvoid(green, 30.0, wall_list)
    green_nav.active_behaviours.append(wallavoid)

    ### Main loop ###
    ticks = 0
    TARGET_FREQ = 100
    while 1:
        for event in pygame.event.get():
            if event.type in [QUIT, MOUSEBUTTONDOWN]:
                pygame.quit()
                sys.exit()

        # Update steering targets every so often
        ticks += 1
        if ticks == TARGET_FREQ:
            # Green target
            x_new = randint(30, sc_width-30)
            y_new = randint(30, sc_height-30)
            new_pos = Point2d(x_new,y_new)
            obj[1].pos = new_pos
            del green_nav.active_behaviours[0]
            new_beh = Flee(green, new_pos)
            green_nav.active_behaviours.insert(0, new_beh)
            ticks = 0

        # Update Vehicles (via manually calling each move() method)
        force = green_nav.update()

        # Update Sprites (via pygame sprite group update)
        allsprites.update(UPDATE_SPEED)

        pygame.time.delay(20)

        # Screen update
        screen.fill(bgcolor)
        allsprites.draw(screen)
        pygame.display.flip()

    pygame.time.delay(2000)
