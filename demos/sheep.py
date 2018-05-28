#!/usr/bin/env python
"""AiBoids flocking demo wiyh pygame rendering."""

# for python3 compat
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import sys
from random import randint

import pygame
from pygame.locals import QUIT, MOUSEBUTTONDOWN

# Note: Adjust this depending on where this file ends up.
# We assume this file lives in aiboids/demos
sys.path.append('..')
from aiboids.point2d import Point2d
from aiboids.vehicle2d import SimpleVehicle2d
from aiboids import pgrender

INF = float('inf')

if __name__ == "__main__":
    # Display set-up
    SCREEN_SIZE = (1024, 768)
    SCREEN, BGCOLOR = pgrender.setup(SCREEN_SIZE, 'FLOCKING/EVADE steering demo.')
    UPDATE_SPEED = 0.5
    BORDER = 30

    # Used to generate random positions and velocities for vehicles
    randpoint = lambda: Point2d(randint(BORDER, SCREEN_SIZE[0]-BORDER), randint(BORDER, SCREEN_SIZE[1]-BORDER))

    # Vehicles (sheep/dog) and obstacle information
    NUMSHEEP = 29
    SHEEP_RADIUS = 20
    DOG_RADIUS = 20
    NUMVEHICLES = NUMSHEEP + 1
    NUMOBSTACLES = 12

    # Amount of time spent flocking
    TIME_PERIOD = 1000

    # Create sprite images
    images = dict()
    images['green'] = pgrender.boid_chevron(SHEEP_RADIUS, (0, 222, 0), (0, 0, 0))
    images['yellow'] = pgrender.boid_chevron(DOG_RADIUS, (222, 222, 0), (0, 0, 0))

    # Randomly generate initial placement for vehicles
    init_pos = [randpoint() for i in range(NUMVEHICLES)]
    init_vel = [(BORDER*UPDATE_SPEED/10)*randpoint().unit() for i in range(NUMVEHICLES)]

    # Flock of sheep and associated pygame sprites
    sheep_list = []
    for i in range(NUMSHEEP):
        sheep = SimpleVehicle2d(init_pos[i], SHEEP_RADIUS, init_vel[i], images['green'])
        sheep_list.append(sheep)

    dog = SimpleVehicle2d(init_pos[NUMSHEEP], DOG_RADIUS, init_vel[NUMSHEEP], images['yellow'])
    # No rule says a dog can't override default physics!
    dog.maxspeed = 10.0

    # List of vehicles and sprites for later use
    vehicles = sheep_list + [dog]
    rgroup = [veh.sprite for veh in vehicles]

    # Static obstacles for pygame (randomly-generated positions)
    obslist, obs_sprites = pgrender.scattered_obstacles(NUMOBSTACLES, 15, SCREEN_SIZE)
    rgroup.extend(obs_sprites)

    # Static Walls for pygame (near screen boundary only)
    wall_list, wall_sprites = pgrender.boundary_walls(SCREEN_SIZE)
    rgroup.extend(wall_sprites)

    # Set-up pygame rendering
    allsprites = pygame.sprite.RenderPlain(rgroup)

##############################
# Navigator set-up starts here
##############################

    # This demo still fails to celebrate its sheep diversity
    # Flock with other sheep, evade the dog, wander around
    for sheep in sheep_list:
        sheep.flockmates = sheep_list
        sheep.navigator.set_steering('FLOCKSEPARATE')
        sheep.navigator.set_steering('FLOCKALIGN')
        sheep.navigator.set_steering('FLOCKCOHESION')
        sheep.navigator.set_steering('EVADE', dog, 180)
        sheep.navigator.set_steering('WANDER', 200, 25, 6)

    # Using flocking on the dog gives convincing chase behaviour
    dog.flockmates = sheep_list
    dog.navigator.set_steering('FLOCKSEPARATE')
    dog.navigator.set_steering('FLOCKALIGN')
    dog.navigator.set_steering('WANDER', 200, 25, 18)

    # All creatures avoid obstacles and walls
    for veh in vehicles:
        veh.navigator.set_steering('OBSTACLEAVOID', obslist)
        veh.navigator.set_steering('WALLAVOID', 25.0, wall_list)

##############################
# Main loop
##############################
    ticks = 0
    align_on = True
    while 1:
        ticks = ticks + 1

        if ticks > TIME_PERIOD:
            ticks = 0
            if align_on:
                align_on = False
                for sheep in sheep_list:
                    sheep.navigator.pause_steering('FLOCKALIGN')
                    sheep.navigator.pause_steering('FLOCKCOHESION')
            else:
                align_on = True
                for sheep in sheep_list:
                    sheep.navigator.resume_steering('FLOCKALIGN')
                    sheep.navigator.resume_steering('FLOCKCOHESION')

        for event in pygame.event.get():
            if event.type in [QUIT, MOUSEBUTTONDOWN]:
                pygame.quit()
                sys.exit()

        # Update Vehicles (this implicitly checks their Navigators)
        for veh in vehicles:
            veh.move(UPDATE_SPEED)

        # Update Sprites (via pygame sprite group update)
        allsprites.update(UPDATE_SPEED)

        # Screen update
        SCREEN.fill(BGCOLOR)

        # Show neighbor links if flocking is currently active
        if align_on:
            for sheep in sheep_list:
                for other in sheep.neighbor_list:
                    if other is not sheep:
                        pygame.draw.line(SCREEN, (0, 128, 0), sheep.pos.ntuple, other.pos.ntuple)

        allsprites.draw(SCREEN)
        pygame.display.flip()
