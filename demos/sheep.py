#!/usr/bin/env python3
"""
sheep.py: Flocking demo with random wandering and obstacle avoidance.

All boids use:
* Random WANDER.
* OBSTACLEAVOID.
* WALLAVOID (walls on screen border with inward normals).

Green "sheep":
* Flocking (ALIGN, COHESION, and SEPARATION), peroidically active/inactive.
* When flocking is active, lines indicate neighbors (other sheep only).
* EVADE the yellow "sheepdog"

Yellow "sheepdog":
* Used SEPARATE and ALIGN (with all "sheep"), but no COHESION.
* Somewhat convincing, but won't win any sheepdog trials.
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
    # Display set-up
    SCREEN_SIZE = (1024, 768)
    SCREEN, BGCOLOR = pgrender.setup(SCREEN_SIZE, 'FLOCKING/EVADE steering demo.')
    UPDATE_SPEED = 0.5
    BORDER = 30

    # Used to generate random positions for vehicles
    randpoint = lambda: Point2d(randint(BORDER, SCREEN_SIZE[0]-BORDER), randint(BORDER, SCREEN_SIZE[1]-BORDER))

    # Vehicles (sheep/dog) and obstacle information
    NUMSHEEP = 29
    SHEEP_RADIUS = 20
    DOG_RADIUS = 20
    WHISKER_FRONT = SHEEP_RADIUS*1.25
    WHISKER_SIDES = SHEEP_RADIUS*1.1
    WALL_WHISKERS = [WHISKER_FRONT*Point2d(1,0),
                     WHISKER_SIDES*Point2d(1,1).unit(),
                     WHISKER_SIDES*Point2d(1,-1).unit()
                     ]
    NUMVEHICLES = NUMSHEEP + 1
    NUMOBSTACLES = 12

    # Amount of time spent flocking
    TIME_PERIOD = 1000

    # Create sprite images
    images = dict()
    images['green'] = pgrender.boid_chevron(SHEEP_RADIUS, (0, 222, 0), (0, 0, 0))
    images['yellow'] = pgrender.boid_chevron(DOG_RADIUS, (222, 222, 0), (0, 0, 0))

    # (radius, mass, maxspeed, maxforce, spritedata)
    SHEEP_DATA = (SHEEP_RADIUS, 1.0, 8.0, 6.0, images['green'])
    DOG_DATA = (DOG_RADIUS, 1.0, 10.0, 6.0, images['yellow'])

    # Flock of sheep and associated pygame sprites
    sheep_list = []
    for i in range(NUMSHEEP):
        sheep = SimpleVehicle2d(randpoint(), Point2d(0,0), *SHEEP_DATA)
        sheep_list.append(sheep)

    #...and your little dog, too!
    dog = SimpleVehicle2d(randpoint(), Point2d(0,0), *DOG_DATA)

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
        veh.navigator.set_steering('WALLAVOID', WALL_WHISKERS, wall_list)

##############################
# Main loop
##############################
    ticks = 0
    align_on = True
    b_running = True
    while b_running:
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
                        # Pygame wants explicit conversion to int
                        this_center = (int(sheep.pos.x), int(sheep.pos.y))
                        other_center = (int(other.pos.x), int(other.pos.y))
                        pygame.draw.line(SCREEN, (0, 128, 0), this_center, other_center)

        allsprites.draw(SCREEN)
        pygame.time.delay(5)
        pygame.display.flip()

        # Check for exit
        for event in pygame.event.get():
            if event.type in [QUIT, MOUSEBUTTONDOWN]:
                b_running = False

    ### End of main loop ###
    pygame.quit()

