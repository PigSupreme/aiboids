# aiboids/vehicle2d.py
"""Wall, Obstacle, Pointmass, and Vehicle classes.

* BaseWall2d: Static, one-directional wall segments.
* BasePointMass2d: Point mass with heading aligned to velocity.
* SimpleObstacle2d: Immobile obstacle with center and bounding radius.
* SimpleVehicle2d: BasePointMass2d with attached Navigator for steering.
* SimpleRigidBody2d: (Experimental) mass with basic rotational physics.

Rendering is done independently of this module, but for convenience each class
provides a set_spriteclass() class method. Once this is set, the constructors
above will use the optional *spritedata* parameter to pass through rendering
information to the sprite class.

Todo:
    Standardize import/update mechanisms for default physics constants.

    Replace _spriteclass access by @property?

    Move SPEED_EPSILON to steering_constants.py and import from there?
"""

# for python3 compat
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import copy

from sys import path
path.append('..')

from aiboids.point2d import Point2d
from aiboids.steering_constants import BASEPOINTMASS2D_DEFAULTS, SIMPLERIGIDBODY2D_DEFAULTS
from aiboids.steering import Navigator

INF = float('inf')

#: A BasePointMass2d has velocity-aligned heading. However, if the speed is
#: almost zero (squared speed is below this threshold), we skip alignment in
#: order to avoid jittery behaviour.
SPEED_EPSILON = .00000000001

class BaseWall2d(object):
    """A base class for static, wall-type obstacles.

    Args:
        center (tuple or Point2d): The center of the wall.
        length (int): Length of the wall in pixels.
        thick (int): Thickness of the wall in pixels.
        f_normal (Point2d): Normal vector out from the front of the wall.
        spritedata (optional): Extra data for rendering; see module notes.

    Note:
        Walls are one-way due to the associated steering behaviours.
    """
    _spriteclass = None
    @classmethod
    def set_spriteclass(cls, spriteclass):
        """Set the default sprite class used to render this type of object."""
        cls._spriteclass = spriteclass

    def __init__(self, center, length, thick, f_normal, spritedata=None):
        # Positional data
        self.pos = Point2d(center[0], center[1])
        self.front = f_normal.unit()
        self.left = self.front.left_normal()
        self.rsq = (length/2)**2
        # Structural data
        self.length = length
        self.thick = thick
        # Wall sprite
        if self.__class__._spriteclass and spritedata is not None:
            self.sprite = self.__class__._spriteclass(self, *spritedata)


class BasePointMass2d(object):
    """A moving object with rectilinear motion and optional sprite.

    Args:
        position (Point2d): Center of mass.
        radius (float): Bounding radius of the object.
        velocity (Point2d): Velocity vector (initial facing is aligned to this).
        spritedata (optional): Extra data for rendering; see module notes.

    This provides a minimal base class for a 2D pointmass with bounding radius
    and heading aligned to velocity. Use move() for rectilinear physics updates
    each cycle (including applying force).
    """
    _spriteclass = None
    @classmethod
    def set_spriteclass(cls, spriteclass):
        """Set the default sprite class used to render this type of object."""
        cls._spriteclass = spriteclass

    _PHYSICS_DEFAULTS = copy.copy(BASEPOINTMASS2D_DEFAULTS)

    def __init__(self, position, radius, velocity, spritedata=None):
        # Basic object physics
        self.pos = copy.copy(position)  # Center of object
        self.radius = radius            # Bounding radius
        self.vel = copy.copy(velocity)  # Current Velocity
        self.accumulated_force = Point2d(0,0) # Do not use static ZERO_VECTOR

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
        if self.__class__._spriteclass and spritedata is not None:
            self.sprite = self.__class__._spriteclass(self, spritedata)


    def accumulate_force(self, force_vector):
        """Add a new force to what's already been accumulated.

        Args:
            force_vector (Point2d): Added to any previously-accumulated force.

        This function is intended to allow multiple sources to exert force on
        an object without needing to compute those forces all at once. Use
        move() below with force_vector=None to apply the resultant force
        accumulated by this method and reset accumulated force to zero.
        """
        self.accumulated_force = self.accumulated_force + force_vector

    def move(self, delta_t=1.0, force_vector=None):
        """Updates rectilinear position, velocity, and acceleration.

        Args:
            delta_t (float): Time increment since last move.
            force_vector (Point2d, optional): Vector force to apply during this update.

        If force_vector is None (the default), we use the force accumulated by
        self.accumulate_force() since the last call to this method, and zero
        out the accumulated force. Otherwise, apply the force_vector given, and
        leave the accumulated force unaffected.

        In any case, the maximum force and resulting velocity are limited by
        maxforce and maxspeed attributes.
        """
        # Update position using current velocity
        self.pos = self.pos + delta_t * self.vel

        # If no force_vector was given, use self-accumulated force.
        if force_vector is None:
            force_vector = copy.copy(self.accumulated_force)
            self.accumulated_force.zero()
        # Don't exceed our maximum force; compute acceleration
        force_vector.truncate(self.maxforce)
        accel = (delta_t/self.mass)*force_vector
        # Compute new velocity, but don't exceed maximum speed.
        self.vel = self.vel + accel
        self.vel.truncate(self.maxspeed)

        # Align heading to match our forward velocity. Note that
        # if velocity is very small, skip this to avoid jittering.
        if self.vel.sqnorm() > SPEED_EPSILON:
            self.front = self.vel.unit()
            self.left = Point2d(-self.front.y, self.front.x)


class SimpleObstacle2d(BasePointMass2d):
    """A static obstacle with center and bounding radius.

    Args:
        position (Point2d): Center of mass.
        radius (float): Bounding radius of the object.
        velocity (Point2d): Velocity vector (initial facing is aligned to this).
        spritedata (optional): Extra data for rendering; see module notes.
    """
    def __init__(self, position, radius, spritedata=None):
        BasePointMass2d.__init__(self, position, radius, Point2d(0,0))
        if self.__class__._spriteclass and spritedata is not None:
            self.sprite = self.__class__._spriteclass(self, spritedata)

    def move(self, delta_t=1.0, force_vector=None):
        pass


class SimpleVehicle2d(BasePointMass2d):
    """Point mass with attached Navigator for steering behaviours.

    Args:
        position (Point2d): Center of mass.
        radius (float): Bounding radius of the object.
        velocity (Point2d): Velocity vector (initial facing is aligned to this).
        spritedata (optional): Extra data for rendering; see module notes.

    Use this class for basic steering.Navigator functionality. Calling move()
    will get the current steering force from the Navigator and apply it.
    """
    def __init__(self, position, radius, velocity, spritedata=None):
        BasePointMass2d.__init__(self, position, radius, velocity, spritedata)
        self.navigator = Navigator(self)

    def move(self, delta_t=1.0, force_vector=None):
        """Compute steering force (via Navigator); update rectilinear motion."""
        self.navigator.update(delta_t)
        BasePointMass2d.move(self, delta_t, self.navigator.steering_force)


class SimpleRigidBody2d(BasePointMass2d):
    """Moving object with linear and angular motion, with optional sprite.

    Notes:
        Still experimental!

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

        if self.__class__._spriteclass and spritedata is not None:
            self.sprite = self.__class__._spriteclass(self, *spritedata)

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
        self.pos = self.pos + delta_t * self.vel

        # Apply force, if any...
        if force_vector:
            # Don't exceed our maximum force; compute acceleration/velocity
            force_vector.truncate(self.maxforce)
            accel = (delta_t/self.mass)*force_vector
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
    """Change default physics parameters for children of BasePointMass2d.

    Todo:
        Standardize this function.
    """
    available = BasePointMass2d._PHYSICS_DEFAULTS.keys()
    for (default, value) in kwargs.items():
        if default in available and value > 0:
            BasePointMass2d._PHYSICS_DEFAULTS[default] = value
        else:
            print('Warning: Physics default %s is unavailable.' % default)

if __name__ == "__main__":
    print("Two-Dimensional Vehicle/Obstacle Classes and Functions. Import this elsewhere.")
