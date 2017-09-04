# vehicle2d.py
"""Module containing Vehicle/Obstacle classes, for use with Pygame.

Todo:
    Write a better docstring for this module.
"""

# for python3 compat
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import sys
sys.path.append('..')

import os, pygame, copy
try:
    from pygame.locals import RLEACCEL
except ImportError:
    pass

INF = float('inf')

from aiboids.point2d import Point2d

from aiboids.steering import SteeringBehavior

from aiboids.steering_constants import BASEPOINTMASS2D_DEFAULTS, SIMPLERIGIDBODY2D_DEFAULTS

# Point2d functions return radians, but pygame wants degrees. The negative
# is needed since y coordinates increase downwards on screen. Multiply a
# math radians result by SCREEN_DEG to get pygame screen-appropriate degrees.
SCREEN_DEG = -57.2957795131

#: A BasePointMass2d has velocity-aligned heading. However, if the speed is
#: almost zero (squared speed is below this threshold), we skip alignment in
#: order to avoid jittery behaviour.
SPEED_EPSILON = .000000001

def load_pygame_image(name, colorkey=None):
    """Loads image from current working directory for use in pygame.

    Args:
        name (string): Image file to load; must be pygame-compatible.
        colorkey: (pygame.Color): Used to set a background color for this image
            that will be ignored during blitting. If set to -1, the upper-left
            pixel color will be used as the background color.

    Returns:
        (pygame.Surface, pygame.Rect): A pygame (Surface, Rect) containing the
        image information. For performance(?), Surface is the same format as
        the pygame display, with the alpha channel removed.

    TODO:
        This function is imported by the demos, but perhaps there is a better
        location for it?
    """
    imagefile = os.path.join(os.getcwd(), name)
    try:
        image_surf = pygame.image.load(imagefile)
    except pygame.error as message:
        print('Error: Cannot load image file: %s' % name)
        print('Current working directory is: %s' % os.getcwd())
        raise SystemExit(message)

    # This converts the surface for maximum blitting performance,
    # including removal of any alpha channel:
    image_surf = image_surf.convert()

    # This sets the background (ignored during blit) color:
    if colorkey is not None:
        if colorkey is -1:
            colorkey = image_surf.get_at((0,0))
        image_surf.set_colorkey(colorkey, RLEACCEL)
    return image_surf, image_surf.get_rect()

class BaseWall2d(object):
    """A base class for static wall-type obstacles.

    Args:
        center (tuple or Point2d): The center of the wall in screen coordinates.
        length (int): Length of the wall in pixels.
        thick (int): Thickness of the wall in pixels.
        f_normal (Point2d): Normal vector out from the front of the wall.
        color (pygame.Color, optional): Color for rendering. Defaults to (0,0,0)
    """

    class BaseWall2dSprite(pygame.sprite.Sprite):
        """Pygame Sprite for rendering BaseWall2d objects."""

        def __init__(self, owner, color=None):
            # Must call pygame's Sprite.__init__ first!
            pygame.sprite.Sprite.__init__(self)

            # Set-up sprite image
            self.image = pygame.Surface((owner.length, owner.thick))
            self.image.set_colorkey((255,0,255))
            if color == None:
                self.color = (0,0,0)
            self.image.fill(self.color)
            self.rect = self.image.get_rect()

            # Put into place for rendering
            self.image = pygame.transform.rotate(self.image, owner.theta)
            self.rect = self.image.get_rect()
            self.rect.center = owner.pos[0], owner.pos[1]

        def update(self, delta_t=1.0):
            """Update placeholder for pygame.Sprite parent class. Does nothing."""
            pass

    def __init__(self, center, length, thick, f_normal, color=None):
        # Positional data
        self.pos = Point2d(center[0], center[1])
        self.theta = f_normal.angle()*SCREEN_DEG -90
        self.front = f_normal.unit()
        self.left = self.front.left_normal()
        self.rsq = (length/2)**2

        self.length = length
        self.thick = thick

        # Wall sprite
        self.sprite = BaseWall2d.BaseWall2dSprite(self, color)


class PointMass2dSprite(pygame.sprite.Sprite):
    """A Pygame sprite used to display a BasePointMass2d object."""

    def __init__(self, owner, img_surf, img_rect):
        # Must call pygame's Sprite.__init__ first!
        pygame.sprite.Sprite.__init__(self)

        self.owner = owner

        # Pygame image information for blitting
        self.orig = img_surf
        self.image = img_surf
        # This lets us share image sources!
        self.rect = img_rect.copy()
        self.rect.center = owner.pos[0], owner.pos[1]
        # Only needed if we're use pygame Sprite collision
        self.radius = owner.radius

    def update(self, delta_t=1.0):
        """Called by pygame.Group.update() to redraw this sprite."""
        owner = self.owner
        # Update position
        self.rect.center = owner.pos[0], owner.pos[1]
        # Rotate for blitting
        theta = owner.front.angle()*SCREEN_DEG
        center = self.rect.center
        self.image = pygame.transform.rotate(self.orig, theta)
        self.rect = self.image.get_rect()
        self.rect.center = center

class BasePointMass2d(object):
    """A moving object with rectilinear motion and optional sprite.

    Parameters
    ----------
    position: Point2d
        Center of mass, in screen coordinates.
    radius: float
        Bounding radius of the object.
    velocity: Point2d
        Velocity vector, in screen coordinates. Initial facing matches this.
    spritedata: list or tuple, optional
        Extra data used to create an associate sprite. See notes below.

    Notes
    -----
    This provides a minimal base class for a pointmass with bounding radius
    and heading aligned to velocity. Use move() for physics updates each
    cycle (including applying force).

    As we typically will be rendering these objects within some environment,
    the constructor provides an optional spritedata parameter that can be used
    to create an associated sprite. This is currently implemented using the
    PointMass2dSprite class above (derived from pygame.sprite.Sprite), but
    can be overridden by changing the _spriteclass attribute.
    """
    
    _spriteclass = PointMass2dSprite
    """Default sprite class to use for rendering."""
    
    _PHYSICS_DEFAULTS = copy.copy(BASEPOINTMASS2D_DEFAULTS)

    def __init__(self, position, radius, velocity, spritedata=None):
        # Basic object physics
        self.pos = copy.copy(position)  # Center of object
        self.radius = radius            # Bounding radius
        self.vel = copy.copy(velocity)  # Current Velocity
        self.accumulated_force = Point2d(0,0)

        # Normalized front vector in world coordinates.
        # This stays aligned with the object's velocity (using move() below)
        try:
            self.front = velocity.unit()
        except ZeroDivisionError:
            # If velocity is <0,0>, set facing to screen upwards
            self.front = Point2d(0,-1)
        self.left = Point2d(-self.front[1], self.front[0])

        # Movement constraints (defaults from steering_constants.py)
        ## TODO: Put these in the function argument, perhaps as **kwargs
        self.mass = BasePointMass2d._PHYSICS_DEFAULTS['MASS']
        self.maxspeed = BasePointMass2d._PHYSICS_DEFAULTS['MAXSPEED']
        self.maxforce = BasePointMass2d._PHYSICS_DEFAULTS['MAXFORCE']
        if spritedata is not None:
            self.sprite = BasePointMass2d._spriteclass(self, *spritedata)

    def accumulate_force(self, force_vector):
        """Add a new force to what's already been accumulated.

        Parameters
        ----------
        force_vector: Point2d
            This is added to the previously-accumulated force.

        Notes
        -----
        This function is intended to allow multiple sources to exert force on
        this object without needing to compute those forces all at once.
        Use move() below with force_vector=None to apply the resultant force
        accumulated by this method and reset accumulated force to zero.
        """
        self.accumulated_force = self.accumulated_force + force_vector

    def move(self, delta_t=1.0, force_vector=None):
        """Updates position, velocity, and acceleration.

        Parameters
        ----------
        delta_t: float
            Time increment since last move.
        force_vector: Point2d or None (default)
            Vector force to apply during this update; see Notes below.

        Notes
        -----
        If force_vector is None (the default), we use the force accumulated
        by self.accumulate_force() since the last call to this method, and
        zero out the accumulated force. Otherwise, apply the force_vector as
        given, and leave the accumulated force unaffected.

        In any case, the maximum force and resulting velocity are limited by
        maxforce and maxspeed attributes.

        The original vehicle classes did not need accumulate_force(), since
        their SteeringBehavior takes care of multiple forces. Less intelligent
        class (such as spring masses) that lack such a manager-type class can
        use the base accumulate/move methods without needing extra code.
        """
        # Update position using current velocity
        self.pos = self.pos + self.vel.scm(delta_t)

        # If no force_vector was given, use self-accumulated force.
        if force_vector is None:
            force_vector = copy.copy(self.accumulated_force)
            self.accumulated_force.zero()
        # Don't exceed our maximum force; compute acceleration
        force_vector.truncate(self.maxforce)
        accel = force_vector.scm(delta_t/self.mass)
        # Compute new velocity, but don't exceed maximum speed.
        self.vel = self.vel + accel
        self.vel.truncate(self.maxspeed)

        # Align heading to match our forward velocity. Note that
        # if velocity is very small, skip this to avoid jittering.
        if self.vel.sqnorm() > SPEED_EPSILON:
            self.front = self.vel.unit()
            self.left = Point2d(-self.front[1], self.front[0])

class SimpleVehicle2d(BasePointMass2d):
    """Point mass with steering behaviour."""

    def __init__(self, position, radius, velocity, spritedata=None):
        BasePointMass2d.__init__(self, position, radius, velocity, spritedata)
        # Steering behavior class for this object.
        self.steering = SteeringBehavior(self)

    def move(self, delta_t=1.0):
        """Compute steering force and update rectilinear motion."""
        force = self.steering.compute_force()
        BasePointMass2d.move(self, delta_t, force)

class SimpleObstacle2d(BasePointMass2d):
    """A static obstacle with center and bounding radius."""

    def __init__(self, position, radius, spritedata=None):
        BasePointMass2d.__init__(self, position, radius, Point2d(0,0), spritedata)

    def move(self, delta_t=1.0):
        pass
        
class SimpleRigidBody2d(BasePointMass2d):

    """Moving object with linear and angular motion, with optional sprite.

    Notes
    -----

    Although this isn't really a point mass in the physical sense, we inherit
    from BasePointMass2d in order to avoid duplicating or refactoring code.
    """
    
    # Additional defaults are added to parent class (BasePointMass2d) 
    BasePointMass2d._PHYSICS_DEFAULTS.update(**SIMPLERIGIDBODY2D_DEFAULTS)
    
    def __init__(self, position, radius, velocity, beta, omega, spritedata=None):

        # Use parent class for non-rotational stuff
        BasePointMass2d.__init__(self, position, radius, velocity, spritedata)

        # Rotational inertia and rotational velocity (degrees[??] per time)
        self.inertia = BasePointMass2d._PHYSICS_DEFAULTS['INERTIA']
        self.omega = omega
        self.maxomega = BasePointMass2d._PHYSICS_DEFAULTS['MAXOMEGA']
        self.maxtorque = BasePointMass2d._PHYSICS_DEFAULTS['MAXTORQUE']

        # Adjust facing (beta is measured relative to direction of velocity)
        self.front = self.front.rotated_by(beta)
        self.left = self.front.left_normal()

        if spritedata is not None:
            self.sprite = PointMass2dSprite(self, *spritedata)

    def move(self, delta_t=1.0, force_vector=None):
        """Updates position, velocity, and acceleration.

        Parameters
        ----------
        delta_t: float
            Time increment since last move.
        force_vector: Point2d, optional
            Constant force during this update.

        Note
        ----
        We must override BasePointMass2d.move() in order to avoid aligning
        our heading with forward velocity.
        """

        # Update position using current velocity
        self.pos = self.pos + self.vel.scm(delta_t)

        # Apply force, if any...
        if force_vector:
            # Don't exceed our maximum force; compute acceleration/velocity
            force_vector.truncate(self.maxforce)
            accel = force_vector.scm(delta_t/self.mass)
            self.vel = self.vel + accel
        # ..but don't exceed our maximum speed
        self.vel.truncate(self.maxspeed)

    def rotate(self, delta_t=1.0, torque=0):
        """Updates heading, angular velocity, and torque.

        Parameters
        ----------
        delta_t: float
            Time increment since last rotate.
        torque: float, optional
            Constant torque during this update.
        """

        # Update current facing
        self.front = self.front.rotated_by(self.omega).unit()
        self.left = self.front.left_normal()

        # Clamp to maximum torque, then compute angular acceleration...
        torque = max(min(torque, self.maxtorque), -self.maxtorque)
        alpha = torque*delta_t/self.inertia

        # ...and apply, but don't exceed our maximum angular velocity
        omega = self.omega + alpha
        self.omega = max(min(omega, self.maxomega), -self.maxomega)

def set_physics_defaults(**kwargs):
    """Change default physics parameters for children of BasePointMass2d."""
    available = BasePointMass2d._PHYSICS_DEFAULTS.keys()
    for (default, value) in kwargs.items():
        if default in available and value > 0:
            BasePointMass2d._PHYSICS_DEFAULTS[default] = value
        else:
            print('Warning: Physics default %s is unavailable.' % default)
            
if __name__ == "__main__":
    print("Two-Dimensional Vehicle/Obstacle Classes and Functions. Import this elsewhere.")
