# aiboids/pgrender.py
"""
Utility functions/classes for rendering AiBoids objects with pygame.

See aiboids/demos/sheep.py for example usage.
"""
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import os
from random import shuffle
from sys import path

import pygame

path.append('..')

from aiboids.point2d import Point2d
from aiboids.vehicle2d import BaseWall2d, BasePointMass2d, SimpleObstacle2d, SimpleVehicle2d

# Point2d functions return radians, but pygame wants degrees. The negative
# is needed since y coordinates increase downwards on screen. Multiply a
# math radians result by SCREEN_DEG to get pygame screen-appropriate degrees.
SCREEN_DEG = -57.2957795131

# Color defaults
BG_COLOR = (79, 148, 205)
WALL_COLOR = (0, 0, 0)
DRYWALL_COLOR = (211, 211, 211)

def load_pygame_image(name, colorkey=None):
    """Loads image from current working directory for use in pygame.

    Args:
        name (string): Image file to load; must be pygame-compatible.
        colorkey: (pygame.Color): Used to set a background color for this image
            that will be ignored during blitting. If set to -1, the upper-left
            pixel color will be used as the background color.

    Returns:
        (pygame.Surface): Image for blitting, converted to the same format as
        the pygame display. Alpha channel is removed (check this).
    """
    imagefile = os.path.join(os.getcwd(), name)
    try:
        image_surf = pygame.image.load(imagefile)
    except pygame.error as message:
        print('Error: Cannot load image file: %s' % name)
        print('Current working directory is: %s' % os.getcwd())
        raise RuntimeError(message)

    # This converts the surface for maximum blitting performance,
    # including removal of any alpha channel:
    image_surf = image_surf.convert()

    # This sets the background (ignored during blit) color:
    if colorkey is not None:
        if colorkey is -1:
            colorkey = image_surf.get_at((0,0))
        image_surf.set_colorkey(colorkey, pygame.locals.RLEACCEL)
    return image_surf


class BaseWall2dSprite(pygame.sprite.Sprite):
    """Pygame Sprite for rendering BaseWall2d objects.

    Args:
        owner (BaseWall2d): The object represented by this sprite.
        wall (pygame.Color): Wall color used for rendering.
        panel (pygame.Color): Color for the front surface of the wall.
    """
    def __init__(self, owner, wall=WALL_COLOR, panel=DRYWALL_COLOR):
        # Must call pygame's Sprite.__init__ first!
        pygame.sprite.Sprite.__init__(self)

        # Set-up sprite image
        self.image = pygame.Surface((owner.length, owner.thick))
        self.image.fill(wall)
        # Front panel for visualizing orientation
        pygame.draw.line(self.image, panel, (0,1), (owner.length,1), 2)

        # Put into place for rendering
        self.rect = self.image.get_rect()
        self.theta = owner.front.angle()*SCREEN_DEG - 90
        self.image = pygame.transform.rotate(self.image, self.theta)
        self.rect = self.image.get_rect()
        midwall = owner.pos - (owner.thick/2.0)*owner.front
        self.rect.center = midwall.ntuple

    def update(self, delta_t=1.0):
        """Update placeholder for pygame.Sprite parent class. Does nothing."""
        pass


class PointMass2dSprite(pygame.sprite.Sprite):
    """A Pygame sprite used to display a BasePointMass2d object.

    Args:
        owner (BaseWall2d): The object represented by this sprite.
        img_surf (pygame.Surface): Base image to use for rendering; see below.

    Notes:
        img_surf is the unscaled, non-rotated image used to draw the sprite for
        this object. Format and orientation should match the display mode. The
        front/left left of the image should face screen right/up, respectively.
        The center of the image is rended at the pointmass' center position.
        A single base image can be shared between different sprites.
    """
    def __init__(self, owner, img_surf):
        # Must call pygame's Sprite.__init__ first!
        pygame.sprite.Sprite.__init__(self)
        self.owner = owner
        # Pygame image information for blitting
        self.orig = img_surf
        self.image = img_surf
        self.rect = img_surf.get_rect().copy()
        self.rect.center = owner.pos[0], owner.pos[1]
        # Only needed if we use pygame Sprite collision
        self.radius = owner.radius

    def update(self, delta_t=1.0):
        """Called by pygame.Group.update() to redraw this sprite.

        Args:
            delta_t (float): Elapsed time since last update; currently unused.
        """
        owner = self.owner
        # Update position
        self.rect.center = owner.pos[0], owner.pos[1]
        # Rotate for blitting
        theta = owner.front.angle()*SCREEN_DEG
        center = self.rect.center
        self.image = pygame.transform.rotate(self.orig, theta)
        self.rect = self.image.get_rect()
        self.rect.center = center

###############################################################
### Set default sprite classes
###############################################################
BaseWall2d.set_spriteclass(BaseWall2dSprite)
BasePointMass2d.set_spriteclass(PointMass2dSprite)
SimpleVehicle2d.set_spriteclass(PointMass2dSprite)
SimpleObstacle2d.set_spriteclass(PointMass2dSprite)

###############################################################
### Convenience functions for demos
###############################################################
def boundary_walls(sc_size, thick=10):
    """Convenience function to generate walls/sprites near the screen border.

    Args:
        sc_size (int, int): Width, height of the display area, in pixels.
        thick (int, optional): Thickness of walls, in pixels.

    Returns:
        (list, list): List of BaseWall2d, List of BaseWall2dSprite.
    """
    sc_width, sc_height = sc_size
    sdata = [WALL_COLOR, DRYWALL_COLOR]
    wall_list = [BaseWall2d((sc_width//2, thick), sc_width, thick, Point2d(0,1), sdata),
                 BaseWall2d((sc_width//2, sc_height-thick), sc_width, thick, Point2d(0,-1), sdata),
                 BaseWall2d((thick, sc_height//2), sc_height, thick, Point2d(1,0), sdata),
                 BaseWall2d((sc_width-thick,sc_height//2), sc_height, thick, Point2d(-1,0), sdata)
                ]
    wall_sprites = [wall.sprite for wall in wall_list]
    return wall_list, wall_sprites

def scattered_obstalces(numobs, radius, obs_image, sc_size):
    """Convenience function for randomly-scattered obstacles and their sprites.

    Args:
        numobs (positive int): Number of obstacles to generate.
        radius (positive float): Obstacle radius.
        obs_image (pygame.Surface): Obstacle image (shared among instances).
        sc_size (int, int): Width, height of the display area, in pixels.

    Returns:
        (list, list): List of SimpleObstacle2d, List of SimpleObstacle2d.

    Notes:
        By default, this uses the same sprite class as PointMass2d.
    """
    sc_width, sc_height = sc_size
    yoffset = sc_height//(numobs+2)
    yvals = list(range(yoffset, sc_height-yoffset, yoffset))
    shuffle(yvals)
    obslist = list()
    for i in range(numobs):
        offset = (i+1)/(numobs+1)
        rany = yvals[i]
        new_pos = Point2d(offset*sc_width, rany)
        obstacle = SimpleObstacle2d(new_pos, radius, obs_image)
        obslist.append(obstacle)
    obs_sprites = [obs.sprite for obs in obslist]
    return obslist, obs_sprites

def setup(screensize, caption_text="AiBoids Demo"):
    """Convenience function for pygame setup boilerplate.

    Args:
        screensize (int, int): Passed through to pygame.display.set_mode().
        caption_text (string, optional): Caption for pygame window.

    Returns:
        (pygame.Surface, pygame.Color): Display surface and background color.
    """
    pygame.init()
    screen = pygame.display.set_mode(screensize)
    pygame.display.set_caption(caption_text)
    bgcolor = BG_COLOR
    return screen, bgcolor

if __name__ == "__main__":
    print('Rendering functions for use with pygame. Import elsewhere.')
