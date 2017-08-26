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
from vehicle2d import BasePointMass2d

ZERO_VECTOR = Point2d(0,0)


class SteeringBehaviour(object):
    def __init__(self, owner):
        """Base class for all steering behaviours.
        
        Args:
            owner (vehicle): Compute the steering for this vehicle.
            
        Subclasses (actual behaviours) should call this method using:
        
        >>> SteeringBehaviour.__init__(self, owner)
        
        Where owner is the vehicle that will use this behaviour.
        """
        self.owner = owner
        
    def force(self, delta_t):
        """Compute the owner's steering force for this behaviour."""
        raise NotImplementedError
    
    def set_params(self, *args, **kwargs):
        """Used by the owner/Navigator to change per-instance values."""
        raise NotImplementedError

class Seek(SteeringBehaviour):
    def __init__(self, owner, target):
        SteeringBehaviour.__init__(self, owner)
        self.target = target
        
    def force(self):
        owner = self.owner
        targetvel = (self.target - owner.pos).unit()
        targetvel = targetvel.scm(owner.maxspeed)
        return targetvel - owner.vel

class Navigator(object):
    
    def __init__(self, vehicle):
        """Helper class for managing steering behaviours."""
        self.vehicle = vehicle
        self.steering_force = Point2d(0,0)
        self.active_behaviours = list()
        # TODO: Give the owner vehicle a reference to its Navigator
    
    def update(self, delta_t=1.0):
        # TODO: Option for budgeted force; choose this in __init__()
        self.compute_force_simple()
        self.vehicle.move(1.0, self.steering_force)
    
    def compute_force_simple(self):
        """Updates the current steering force using all active behaviors.

        Note:
            Since the vehicle classes are expected to limit the maximum force
            applied, this function no longer does so.
        """
        self.steering_force.zero()
        owner = self.vehicle
        # If any flocking is active, determine neighbors first
        #if self.flocking is True:
        #    self.flag_neighbor_vehicles(self.flockmates)
        # Iterate over active behaviours and accumulate force from each
        for behaviour in self.active_behaviours:
            self.steering_force += behaviour.force()

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
    numobs = 0
    total = 2

    # Sprite images and pygame rectangles
    img = list(range(total))
    rec = list(range(total))

    # Load vehicle images
    img[0], rec[0] = load_pygame_image('../images/gpig.png', -1)

    # Steering behaviour target images (generated here)
    for i in range(numveh, 2*numveh):
        img[i] = pygame.Surface((5,5))
        rec[i] = img[i].get_rect()

    # Static obstacle image (shared among all obstacles)
#    obs_img, obs_rec = load_pygame_image('../images/circle.png', -1)
#    for i in range(2*numveh, 2*numveh + numobs):
#        img[i], rec[i] = obs_img, obs_rec

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

#    # Static obstacles for pygame (randomly-generated positions)
#    yoffset = sc_height//(numobs+1)
#    yvals = list(range(yoffset, sc_height-yoffset, yoffset))
#    shuffle(yvals)
#    for i in range(2*numveh, 2*numveh + numobs):
#        offset = (i+1.0-2*numveh)/(numobs+1)
#        rany = yvals[i-2*numveh]
#        new_pos = Point2d(offset*sc_width, rany)
#        obstacle = SimpleObstacle2d(new_pos, 10, (img[i], rec[i]))
#        obj.append(obstacle)
#        rgroup.append(obstacle.sprite)
#    # This gives a convenient list of (non-wall) obstacles for later use
#    obslist = obj[2*numveh:]
#
#    # Static Walls for pygame (screen border only)
#    wall_list = (BaseWall2d((sc_width//2, 10), sc_width-20, 4, Point2d(0,1)),
#                 BaseWall2d((sc_width//2, sc_height-10), sc_width-20, 4, Point2d(0,-1)),
#                 BaseWall2d((10, sc_height//2), sc_height-20, 4, Point2d(1,0)),
#                 BaseWall2d((sc_width-10,sc_height//2), sc_height-20, 4, Point2d(-1,0)))
#    obj.extend(wall_list)
#    for wall in wall_list:
#        rgroup.append(wall.sprite)

    # Set-up pygame rendering
    allsprites = pygame.sprite.RenderPlain(rgroup)

    ### Vehicle steering behavior defined below ###
#    # All vehicles will avoid obstacles and walls
#    for i in range(3):
#        obj[i].steering.set_target(AVOID=obslist, WALLAVOID=[30, wall_list])
    ### End of vehicle behavior ###



    # Green (SEEK)
    green_nav = Navigator(green)
    newseek = Seek(green, obj[1].pos)
    green_nav.active_behaviours.append(newseek)

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
            newseek = Seek(green, new_pos)
            green_nav.active_behaviours= [newseek]
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
