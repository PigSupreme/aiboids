#!/usr/bin/env python
"""AiBoids flocking demo wiyh pygame rendering."""

# for python3 compat
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import sys, pygame
from pygame.locals import QUIT, MOUSEBUTTONDOWN
from random import randint

# Note: Adjust this depending on where this file ends up.
sys.path.append('..')
from aiboids.point2d import Point2d
from aiboids.vehicle2d import SimpleVehicle2d
from aiboids import pgrender

INF = float('inf')

if __name__ == "__main__":
    # Display set-up
    SCREEN_SIZE = (800,640)
    screen, bgcolor = pgrender.setup(SCREEN_SIZE, 'FLOCKING/EVADE steering demo.')
    UPDATE_SPEED = 0.15
    BORDER = 30

    # Used to generate random positions and velocities for vehicles
    randpoint = lambda: Point2d(randint(BORDER, SCREEN_SIZE[0]-BORDER), randint(BORDER, SCREEN_SIZE[1]-BORDER))

    # Number of vehicles and obstacles
    numsheep = 16
    numveh = numsheep + 1   # Extra is the dog
    numobs = 10

    # Load images
    images = dict()
    images['green'] = pgrender.load_pygame_image('../images/gpig.png', -1)
    images['yellow'] = pgrender.load_pygame_image('../images/ypig.png', -1)
    images['obstacle'] = pgrender.load_pygame_image('../images/circle.png', -1)

    # Randomly generate initial placement for vehicles
    init_pos = [randpoint() for i in range(numveh)]
    init_vel = [(BORDER*UPDATE_SPEED/10)*randpoint().unit() for i in range(numveh)]

    # Flock of sheep and associated pygame sprites
    sheep_list = []
    for i in range(numsheep):
        sheep = SimpleVehicle2d(init_pos[i], 40, init_vel[i], images['green'])
        sheep_list.append(sheep)
    # ...and your little dog, too!
    dog = SimpleVehicle2d(init_pos[numsheep], 40, init_vel[numsheep], images['yellow'])

    # List of vehicles and sprites for later use
    vehicles = sheep_list + [dog]
    rgroup = [veh.sprite for veh in vehicles]

    # Static obstacles for pygame (randomly-generated positions)
    obslist, obs_sprites = pgrender.scattered_obstalces(numobs, 10, images['obstacle'], SCREEN_SIZE)
    rgroup.extend(obs_sprites)

    # Static Walls for pygame (near screen boundary only)
    wall_list, wall_sprites = pgrender.boundary_walls(SCREEN_SIZE)
    rgroup.extend(wall_sprites)

    # Set-up pygame rendering
    allsprites = pygame.sprite.RenderPlain(rgroup)

##############################
# Navigator set-up starts here
##############################

    # This demo fails to celebrate its sheep diversity
    # Flock with other sheep and evade the dog
    for sheep in sheep_list:
        sheep.flockmates = sheep_list
        sheep.navigator.set_steering('FLOCKSEPARATE')
        sheep.navigator.set_steering('FLOCKALIGN')
        sheep.navigator.set_steering('FLOCKCOHESION')
        sheep.navigator.set_steering('EVADE', dog, 180)
        sheep.navigator.set_steering('WANDER', 250, 20, 5)

    # No rule says a dog can't override default physics!
    dog.maxspeed, dog.radius = 10.0, 50
    dog.flockmates = sheep_list
    dog.navigator.set_steering('FLOCKSEPARATE')
    dog.navigator.set_steering('FLOCKALIGN')
    dog.navigator.set_steering('WANDER', 200, 25, 6)

    # All vehicles avoid obstacles and walls
    for veh in vehicles:
        veh.navigator.set_steering('OBSTACLEAVOID', obslist)
        veh.navigator.set_steering('WALLAVOID', 25.0, wall_list)

##############################
# Main loop
##############################
    while 1:
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
        screen.fill(bgcolor)
        allsprites.draw(screen)
        pygame.display.flip()
