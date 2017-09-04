# steering.py
"""Module containing PyBOID steering behavior functions.

Each type of behaviour needs a force_foo() function to compute the actual
steering force. The first argument ("owner") is the vehicle that is being
steered. Any number of additional arguments are permitted. This allows the
SteeringBehaviour.compute_force functions to automatically call each active
behaviour's force_foo() function with appropriate arguments.

Each behaviour also needs a activate_foo() function. The first argument
("steering") is an instance of SteeringBehaviour owned by the given vehicle;
additional arguments are intended to be stored within the SteeringBehaviour
instance and then passed to the corresponding force_foo() each update. See
the SEEK code for a simple example.

Many behaviours use constant values, imported from steering_constants.py and
assigned to local constants within this module. These are chosen based on the
the physics defaults (also in steering_constants) to give reasonable results,
but can be overridden with steering.FOO_CONSTANT = new_value. See the sheepdog
demo for an example of this.

Importing this module will automatically generate a BEHAVIOUR_LIST containing
all behaviours that follow the conventions above. This makes it very easy to
add additional behaviours with minimal changes to the existing code (besides
writing the force/activate functions, SteeringBehaviour.PRIORITY_LIST would
need to be modified with any new behaviours if we use budgeted force).

TODO: set/pause/resume/stop behaviour functions always call set_priorities()
regardless of whether budgeted force is actually used. Since we're currently
using budgeted force all the time, this issue is pretty much unimportant.

TODO: Updates to self.flocking are handled through set_priorities(), which is
a sensible thing, since set_priorities() is the function that gets called when
there is any kind of behaviour change. Make up our minds whether this is truly
the right approach and change documentation appropriately.
"""

# for python3 compat
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

from sys import path
path.extend(['../vpoints'])
from point2d import Point2d

import logging

# Default constants for the various steering behaviours
from steering_constants import STEERING_DEFAULTS
FLEE_PANIC_SQ = STEERING_DEFAULTS['FLEE_PANIC_SQ']
ARRIVE_DECEL_TWEAK = STEERING_DEFAULTS['ARRIVE_DECEL_TWEAK']
EVADE_PANIC_SQ = STEERING_DEFAULTS['EVADE_PANIC_SQ']
TAKECOVER_STALK_T = STEERING_DEFAULTS['TAKECOVER_STALK_T']
WALLAVOID_WHISKER_SCALE = STEERING_DEFAULTS['WALLAVOID_WHISKER_SCALE']
FOLLOW_ARRIVE_HESITANCE = STEERING_DEFAULTS['FOLLOW_ARRIVE_HESITANCE']
AVOID_MIN_LENGTH = STEERING_DEFAULTS['AVOID_MIN_LENGTH']
AVOID_BRAKE_WEIGHT = STEERING_DEFAULTS['AVOID_BRAKE_WEIGHT']
WAYPOINT_TOLERANCE_SQ = STEERING_DEFAULTS['WAYPOINT_TOLERANCE_SQ']
PATH_EPSILON_SQ = STEERING_DEFAULTS['PATH_EPSILON_SQ']
PATHRESUME_DECAY = STEERING_DEFAULTS['PATHRESUME_DECAY']
FLOCKING_COHESHION_HESITANCE = STEERING_DEFAULTS['FLOCKING_COHESHION_HESITANCE']
FLOCKING_RADIUS_MULTIPLIER = STEERING_DEFAULTS['FLOCKING_RADIUS_MULTIPLIER']
FLOCKING_SEPARATE_SCALE = STEERING_DEFAULTS['FLOCKING_SEPARATE_SCALE']

# Math Constants (for readability)
INF = float('inf')
from math import sqrt
SQRT_HALF = sqrt(0.5)
ZERO_VECTOR = Point2d(0,0)

# Random number generator (used by WANDER only)
from random import Random
rand_gen = Random()
rand_gen.seed()
rand_uni = lambda x: rand_gen.uniform(-x, x)


def force_seek(owner, target):
    """Steering force for SEEK behaviour.

    This is a simple behaviour that directs the owner towards a given point.
    Other, more complex behaviours make use of this.

    Parameters
    ----------
    owner: SimpleVehicle2d
        The vehicle computing this force.
    position: Point2d
        The target point that owner is seeking to.
    """
    targetvel = (target - owner.pos).unit()
    targetvel = targetvel.scm(owner.maxspeed)
    return targetvel - owner.vel

def activate_seek(steering, target):
    """Activate SEEK behaviour."""
    # TODO: Error checking here.
    steering.targets['SEEK'] = (Point2d(*target),)
    return True

def force_flee(owner, target, panic_squared=FLEE_PANIC_SQ):
    """Steering force for FLEE behaviour.

    Another simple behaviour that directs the owner away from a given point.

    Parameters
    ----------
    owner: SimpleVehicle2d
        The vehicle computing this force.
    position: Point2d
        The target point that owner is fleeing from.
    panic_squared: float
        If specified, only compute a flee_force if squared distance
        to the target is less than this value.
    """
    targetvel = (owner.pos - target)
    if 1 < targetvel.sqnorm() < panic_squared:
        targetvel = targetvel.unit().scm(owner.maxspeed)
        return targetvel - owner.vel
    else:
        return ZERO_VECTOR

def activate_flee(steering, target):
    """Activate FLEE behaviour."""
    # TODO: Error checking here.
    # TODO: Provide a way to set individual panic_squared values??
    steering.targets['FLEE'] = (Point2d(*target),)
    return True

def force_arrive(owner, target, hesitance=2.0):
    """Steering force for ARRIVE behaviour.

    This works like SEEK, except the vehicle gradually deccelerates as it
    nears the target position. The optional third parameter controls the
    amount of decceleration.

    Parameters
    ----------
    owner: SimpleVehicle2d
        The vehicle computing this force.
    position: Point2d
        The target point that owner is to arrive at.
    hesistance: float
        Controls the time it takes to deccelerate; higher values give more
        gradual (and slow) decceleration. Suggested values are 1.0 - 10.0.

    """
    target_offset = (target - owner.pos)
    dist = target_offset.norm()
    if dist > 0:
        # The constant on the next line may need tweaking
        speed = dist / (ARRIVE_DECEL_TWEAK * hesitance)
        if speed > owner.maxspeed:
            speed = owner.maxspeed
        targetvel = target_offset.scm(speed/dist)
        return targetvel - owner.vel
    else:
        return ZERO_VECTOR

def activate_arrive(steering, target):
    """Activate ARRIVE behaviour."""
    if len(target) == 2:
        steering.targets['ARRIVE'] = (Point2d(*target),)
    else:
        steering.targets['ARRIVE'] = (Point2d(target[0], target[1]), target[2])
    return True

def force_pursue(owner, prey):
    """Steering force for PURSUE behaviour.

    Similar to SEEK, but lead the prey by estimating its future location,
    based on current velocities.

    Parameters
    ----------
    owner: SimpleVehicle2d
        The vehicle computing this force.
    prey: BasePointMass2d
        The vehicle that owner will pursue.
    """
    prey_offset = prey.pos - owner.pos
    # If prey is in front and moving our way, SEEK to prey's position
    # Compute this using dot products; constant below is cos(10 degrees)
    # TODO: Test this and allow for angles other than 10 degrees
    if prey_offset * prey.front < -0.966 * prey_offset.norm():
        return force_seek(owner, prey.pos)

    # Otherwise, predict the future position of prey, assuming it has a
    # constant velocity. Prediction time is the distance to prey, divided
    # by the sum of our max speed and prey's current speed.
    ptime = prey_offset.norm()/(owner.maxspeed + prey.vel.norm())
    return force_seek(owner, prey.vel.scm(ptime) + prey.pos)

def activate_pursue(steering, prey):
    """Activate PURSUE behaviour."""
    # TODO: Error checking here.
    steering.targets['PURSUE'] = (prey,)
    return True

def force_evade(owner, predator):
    """Steering force for EVADE behaviour.

    Similar to FLEE, but try to get away from the predicted future position
    of the predator. Predators far away are ignored, EVADE_PANIC_SQ is used
    to control the panic distance passed to FLEE.

    Parameters
    ----------
    owner: SimpleVehicle2d
        The vehicle computing this force.
    predator: BasePointMass2d
        The vehicle that owner will pursue.
    """
    predator_offset = predator.pos - owner.pos
    # Predict the future position of predator, assuming it has a constant
    # velocity. Prediction time is the distance to predator divided
    # by the sum of our max speed and predator's current speed.
    ptime = predator_offset.norm()/(owner.maxspeed + predator.vel.norm())
    return force_flee(owner, predator.vel.scm(ptime) + predator.pos, EVADE_PANIC_SQ)

def activate_evade(steering, predator):
    """Activate EVADE behaviour."""
    # TODO: Error checking here.
    steering.targets['EVADE'] = (predator,)
    return True

def force_wander(owner, steering):
    """Steering force for WANDER behavior.

    Parameters
    ----------
    owner: SimpleVehicle2d
        The vehicle computing this force
    steering: SteeringBehavior
        An instance of SteeringBehavior (*not* a vehicle, see note below)

    Note
    ----
    WANDER requires persistant data (specifically, the target of the wander
    circle), so we need access to the SteeringBehavior itself instead of the
    vehicle that owns it.
    """
    params = steering.wander_params
    jitter = params[2]

    # Add a random displacement to previous target and reproject
    target = steering.wander_target + Point2d(rand_uni(jitter), rand_uni(jitter))
    target.normalize()
    target = target.scm(params[1])
    steering.wander_target = target
    return force_seek(owner, owner.pos + target + owner.front.scm(params[0]))

def activate_wander(steering, target):
    """Activate WANDER behaviour."""
    # TODO: Fix arguments, check errors
    steering.wander_params = target
    steering.wander_target = steering.vehicle.front
    # Next line may seem weird, but needed for unified interface.
    steering.targets['WANDER'] = (steering,)
    return True

def force_avoid(owner, obs_list):
    """Steering force for AVOID stationary obstacles behaviour.

    This projects a box in front of the owner and tries to find an obstacle
    for which collision is imminent (not always the closest obstacle). The
    owner will attempt to steer around that obstacle.

    Parameters
    ----------
    owner: SimpleVehicle2d
        The vehicle computing this force.
    obs_list: list of BasePointMass2d
        List of obstacles to check for avoidance.
    """

    # Obstacles closer than this distance will be avoided
    front_d = (1 + owner.vel.norm()/owner.maxspeed)*AVOID_MIN_LENGTH
    front_sq = front_d * front_d

    # Find the closest obstacle within the detection box
    xmin = 1 + front_d
    obs_closest = None
    for obstacle in obs_list:
        # Consider only obstacles that are nearby
        target = obstacle.pos
        diff = target - owner.pos
        if diff.sqnorm() < front_sq:
            # Convert to local coordinates of the owner
            local_x = diff / owner.front # This is an Orthogonal projection
            # Only consider objects in front
            if local_x > 0:
                # Find nearest x-intercept of extended bounding circle
                local_y = diff / owner.left
                expr = owner.radius + obstacle.radius
                xval = local_x - sqrt(expr*expr + local_y*local_y)
                # If this obstacle is closer, update minimum values
                if xval < xmin:
                    xmin, lx, ly = xval, local_x, local_y
                    obs_closest = obstacle

    # If there is a closest obstacle, avoid it
    if obs_closest:
        lr = obs_closest.radius
        lat_scale = (lr - ly)*(2.0 - lr / front_d)
        brake_scale = (lr - lx)*AVOID_BRAKE_WEIGHT
        result = owner.front.scm(brake_scale) + owner.left.scm(lat_scale)
        return result
    else:
        return ZERO_VECTOR

def activate_avoid(steering, target):
    """Activate AVOID behaviour."""
    steering.targets['AVOID'] = (target,)
    # TODO: Fix arguments, check errors
    # Currently we're passing a list of SimpleObstacle2d
    return True

def force_takecover(owner, target, obs_list, max_range, stalk=False):
    """Steering force for TAKECOVER behind obstacle.

    Owner attempts to move to the nearest position that will put an obstacle
    between itself and the target. If no such points are within max_range,
    EVADE the predator instead.

    By setting stalk to True, we'll only hide when in front of the target.
    Stalking allows this vehicle to (somewhat clumsily) sneak up from behind.

    Parameters
    ----------
    owner: SimpleVehicle2d
        The vehicle computing this force.
    target: BasePointMass2d
        The vehicle we try to hide from.
    obs_list: list of BasePointMass2d
        List of obstacles to check for avoidance.
    max_range: float
        Obstacles further than this value are ignored.
    stalk: boolean
        If True, only hide when we are in front of the target.
    """

    # If we're stalking, only hide when we're in front of our target.
    if stalk:
        hide_dir = (owner.pos - target.pos)
        if (hide_dir * target.front)**2 < hide_dir.sqnorm()*TAKECOVER_STALK_T:
            return ZERO_VECTOR

    best_dsq = max_range*max_range
    best_pos = None
    for obs in obs_list:
        # Find the hiding point for this obstacle
        hide_dir = (obs.pos - target.pos).unit()
        hide_pos = obs.pos + hide_dir.scm(obs.radius + owner.radius)
        hide_dsq = (hide_pos - owner.pos).sqnorm()
        # Update distance and position if this obstacle is better
        if hide_dsq < best_dsq:
            best_pos = hide_pos
            best_dsq = hide_dsq

    if best_pos is None:
        return force_evade(owner, target)
    else:
        return force_arrive(owner, best_pos, 1.0)

def activate_takecover(steering, target):
    """Activate TAKECOVER behaviour."""
    # TODO: Error checking
    steering.targets['TAKECOVER'] = target
    return True

def force_wallavoid(owner, whisk_units, whisk_lens, wall_list):
    """Steering force for WALLAVOID behaviour with aribtrary whiskers.

    For each whisker, we find the wall with point of intersection closest
    to the base of the whisker. If such a wall is detected, it contributes a
    force in the direction of the wall normal proportional to the penetration
    depth of the whisker. Total force is the resultant vector sum.

    Parameters
    ----------
    owner: SimpleVehicle2d
        The vehicle computing this force.
    whisk_units: list of Point2d or 2-tuple
        Whisker UNIT vectors in owner's local coordinates (forward is x+).
    whisk_lens: list of positive int or float
        Lengths of whiskers, in same order as whisk_units above.
    wall_list: list of BaseWall2d
        Walls to test for avoidance.
    """

    n = len(whisk_units)
    whisk_front = n*[Point2d(0,0)]
    closest_wall = n*[None]

    # Covert unit vectors for each whisker to global coordinates
    for i in range(n):
        whisker = whisk_units[i]
        unit_whisker = owner.front.scm(whisker[0]) + owner.left.scm(whisker[1])
        whisk_front[i] = unit_whisker
        t_min = whisk_lens[:]

    # Find the closest wall intersecting each whisker
    for wall in wall_list:
        # TODO: Check against wall radius for better efficiency??

        # Numerator of intersection test is the same for all whiskers
        t_numer = wall.front * (wall.pos - owner.pos)
        for i in range(n):
            # Is vehicle in front and whisker tip behind wall's infinite line?
            try:
                t = t_numer / (wall.front * whisk_front[i])
            except ZeroDivisionError:
                # Whisker is parallel to wall in this case, no intersection
                continue
            if 0 < t < t_min[i]:
                # Is the point of intersection actually on the wall segment?
                poi = owner.pos + whisk_front[i].scm(t)
                if (wall.pos - poi).sqnorm() <= wall.rsq:
                    # This is the closest intersecting wall so far
                    closest_wall[i] = wall
                    t_min[i] = t

    # For each whisker, add the force away from the closest wall (if any)
    result = Point2d(0,0)
    for i in range(n):
        if closest_wall[i] is not None:
            depth = whisk_lens[i] - t_min[i]
            result += closest_wall[i].front.scm(depth)

    # Scale by owner radius; bigger objects should tend to stay away
    return result.scm(owner.radius)

def activate_wallavoid(steering, info):
    """Activate WALLAVOID behaviour.

    Note
    ----

    Whisker angles are assumed at 45 degrees; scale is set by steering constants.
    """
    # TODO: Fix arguments, check errors
    # Three whiskers: Front and left/right by 45 degrees
    whiskers = [Point2d(1,0), Point2d(SQRT_HALF, SQRT_HALF), Point2d(SQRT_HALF, -SQRT_HALF)]
    whisker_lengths = [info[0]] + 2*[info[0]*WALLAVOID_WHISKER_SCALE]
    steering.targets['WALLAVOID'] = [whiskers, whisker_lengths, info[1]]
    return True

def force_guard(owner, guard_this, guard_from, aggro):
    """Steering force for GUARD behavior.

    Parameters
    ----------
    owner: SimpleVehicle2d
        The vehicle computing this force.
    guard_this: BasePointMass2d
        The target that owner is guarding.
    guard_from: BasePointMass2d
        The target that owner is guarding against.
    aggro: float
        Value from 0 to 1; controls aggressiveness (see notes below)

    Notes
    -----
    This is a more general version of INTERPOSE. The vehicle will attempt
    to position itself between guard_this and guard_from, at a relative
    distance controlled by aggro. Setting aggro near zero will position near
    guard_this; aggro near 1.0 will position near guard_from.

    The formula is the standard parameterization of a line segment, so we can
    actually set aggro outside of the unit interval.
    """

    # Find the desired position between the two objects as of now:
    target_pos = guard_this.pos
    from_pos = guard_from.pos
    want_pos = target_pos + (from_pos - target_pos).scm(aggro)

    # Predict future positions based on owner's distance/maxspeed to want_pos
    est_time = (want_pos - owner.pos).norm()/owner.maxspeed
    target_pos += guard_this.vel.scm(est_time)
    from_pos += guard_from.vel.scm(est_time)
    want_pos = target_pos + (from_pos - target_pos).scm(aggro)

    return force_arrive(owner, want_pos, 1.0)

def activate_guard(steering, target):
    """Activate GUARD behaviour."""
    steering.targets['GUARD'] = target
    # TODO: Check for errors
    return True

def force_follow(owner, leader, offset):
    """Steering force for FOLLOW the leader at some offset.

    Parameters
    ----------
    owner: SimpleVehicle2d
        The vehicle computing this force.
    leader: BasePointMass2d
        The lead vehicle that the owner is following.
    offset: Point2d
        Offset from leader (in leader's local coordinates, front = +x)
    """

    target_pos = leader.pos + leader.front.scm(offset[0]) + leader.left.scm(offset[1])
    diff = target_pos - owner.pos
    ptime = diff.norm() / (owner.maxspeed + leader.vel.norm())
    target_pos += leader.vel.scm(ptime)
    return force_arrive(owner, target_pos, FOLLOW_ARRIVE_HESITANCE)

def activate_follow(steering, target):
    """Activate FOLLOW behaviour."""
    steering.targets['FOLLOW'] = target
    # TODO: Check for errors
    return True

def force_brake(owner, decay=0.5):
    """Steering force oppoisite of current forward velocity.

    Parameters
    ----------
    owner: SimpleVehicle2d
        The vehicle computing this force.
    decay: float
        Discrete exponential decay constant for speed; 0 < decay < 1.
    """
    speed = owner.vel.norm()
    return owner.vel.scm(-decay * speed)

def activate_brake(steering, target):
    """Activate BRAKE behaviour."""
    # TODO: Error checking here.
    if 0 < target < 1:
        steering.targets['BRAKE'] = (target,)
    else:
        steering.targets['BRAKE'] = (0.5,)
    return True

##############################################
### Path-related behaviours start here     ###
##############################################

class WaypointPath(object):
    """Helper class for managing path-related behaviour, using waypoints.

    Parameters
    ----------
    waypoints: list of Point2d
        Non-empty list of waypoints on this path.
    is_cyclic: boolean
        If set to True, path will automatically cycle. See notes below.

    Notes
    -----
    Instances of WaypointPath should be owned by a SteeringBehaviour, but all
    path-management code is controlled from within this class.

    When using this for vehicle steering, the first waypoint is intended as the
    starting point of some owner vehicle. The vehicle will *not* automatically
    return to this point even if is_cyclic is set to True, so add it manually
    to the end of waypoints if a return trip is needed.

    TODO: It may be helpful to rewrite this class as a generator.
    """

    def __init__(self, waypoints, is_cyclic=False):
        self.oldway = waypoints[0]
        self.waypoints = []
        prev_wp = self.oldway

        # Include only consecutive waypoints that are far enough apart
        for wp in waypoints[1:]:
            if (prev_wp - wp).sqnorm() >= PATH_EPSILON_SQ:
                self.waypoints.append(wp)
                prev_wp = wp

        # Compute initial segment, see Notes on returning to first waypoint
        self.newway = self.waypoints[0]
        self.wpindex = 0

        # Length of this edge and unit vector (oldway to newway)
        offset = self.newway - self.oldway
        self.edgelength = offset.norm()
        self.edgevector = offset.scm(1/self.edgelength)

        self.is_cyclic = is_cyclic

    def reset_from_position(self, start_pos, do_return=False):
        """Reset the next waypoint to the start of this path.

        Parameters
        ----------
        start_pos: Point2d
            The new starting point for the path
        do_return: boolean
            If set to True, start_pos becomes the final waypoint. See Notes.

        Notes
        -----
        As with the __init__() method, start_pos is intended as the current
        location of some vehicle, and is not explicitly added as the first
        waypoint. If the path was previously cyclic, we will not return to
        start_pos by default (but the previous waypoints will still continue
        to cycle). To override this, set do_return=True. However, start_pos
        will not be added explicitly is it is within the threshold given by
        PATH_EPSILON_SQ, because it is close enough to an actual waypoint.
        """
        self.newway = self.waypoints[0]
        self.wpindex = 0
        # TODO: Make sure this works for single edges and start_pos close to
        # the first or last waypoint in a cyclic path.
        # If we're close to the first waypoint, use that wp as start_pos
        if (start_pos - self.newway).sqnorm() < PATH_EPSILON_SQ:
            self.advance()
        if do_return and (start_pos - self.waypoints[-1]).sqnorm() >= PATH_EPSILON_SQ:
            self.waypoints.append(start_pos)

    def advance(self):
        """Update our waypoint to the next one in the path.

        Notes
        -----
        When we advance() from the last waypoint in a non-cyclic path, the
        value of self.newway is set to None. This can be used elsewhere??
        """
        self.oldway = self.newway
        self.wpindex = self.wpindex + 1

        try:
            self.newway = self.waypoints[self.wpindex]
            # Compute new length and unit vector
            offset = self.newway - self.oldway
            self.edgelength = offset.norm()
            self.edgevector = offset.scm(1/self.edgelength)

        # This throws if we are at the last waypoint in the list.
        except IndexError:
            if self.is_cyclic:
                # If cyclic, go back to the first waypoint
                self.wpindex = 0
                self.newway = self.waypoints[0]
                offset = self.newway - self.oldway
                self.edgelength = offset.norm()
                self.edgevector = offset.scm(1/self.edgelength)
            else:
                self.newway = None
                self.edgelength = 0
                self.edgevector = None

    def num_left(self):
        """Returns the number of waypoints remaining in this path.

        Notes
        -----
        For cyclic paths, we always return the total number of waypoints,
        regardless of where we are in the list.
        """
        if self.is_cyclic:
            return len(self.waypoints)
        else:
            return len(self.waypoints) - self.wpindex


def force_waypathtraverse(owner, waypath):
    """Steering force for WAYPATHTRAVERSE behaviour.

    Parameters
    ----------
    owner: SimpleVehicle2d
        The vehicle computing this force.
    waypath: WaypointPath
        Path to be followed by the owner

    Notes
    -----
    This is the simple version; we merely head towards the next waypoint.
    If there is only one waypoint left, we ARRIVE at it. Otherwise, we SEEK.
    """
    # If no waypoint left, exit immediately
    if waypath.newway is None:
        return ZERO_VECTOR

    # If current destination is the last waypoint on this path, ARRIVE
    # at that waypoint
    if waypath.num_left() <= 1:
        return force_arrive(owner, waypath.newway)

    # Otherwise, check if we've reached the next waypoint
    # Note: No force is returned when we switch to the next waypoint
    if (owner.pos - waypath.newway).sqnorm() <= WAYPOINT_TOLERANCE_SQ:
        waypath.advance()
        return ZERO_VECTOR

    # TODO: This is for testing only?
    owner.waypoint = waypath.newway

    return force_seek(owner, waypath.newway)

def activate_waypathtraverse(steering, waypath):
    """Activate WAYPATHTRAVERSE behaviour."""
    # TODO: Error checking here.
    steering.targets['WAYPATHTRAVERSE'] = (waypath,)
    return True

def force_waypathresume(owner, waypath, invk):
    """Steering force for WAYPATHRESUME behaviour.

    Parameters
    ----------
    owner: SimpleVehicle2d
        The vehicle computing this force.
    waypath: WaypointPath
        Path to be followed by the owner
    invk: positive float
        Reciprocal of exponential decay constant. See Notes.

    Notes
    -----
    If the vehicle is off course, this will give a balance between returning
    directly to the current path edge and progressing to the next waypoint.

    If the vehicle has already overshot the next waypoint, we head directly to
    that waypoint, ignoring the path. Otherwise, follow an exponential decay
    curve asymptotic to the path; although this curve doesn't actually pass
    through the waypoint, it makes computations very quick, especially since
    we store invk. Smaller values of invk imply a larger decay rate, and give
    more immediate return to the path.
    """
    # If no waypoint left, exit immediately
    if waypath.newway is None:
        return ZERO_VECTOR

    # This is the remaining direct distance to the next waypoint,
    # using orthogonal projection operator.
    rl = (waypath.newway - owner.pos)/waypath.edgevector

    # If resume target is beyond the next waypoint, SEEK/ARRIVE to waypoint.
    # Otherwise, SEEK (never ARRIVE) to the resume target
    if invk >= rl: # Resume target is beyond the next waypoint
        target = waypath.newway
        # ARRIVE if this is the last waypoint; no further computation neeed
        if waypath.num_left() <= 1:
            return force_arrive(owner, waypath.newway)
    else: # Resume target is between last/next waypoints
        target = waypath.newway + waypath.edgevector.scm(invk - rl)

    # If we reach this part of the code, we must SEEK to either a target on
    # the path or a waypoint that is not the last one in the path. So...
    # Check if we're close enough to the next waypoint to switch.
    # Note: No force is returned when we switch to the next waypoint
    if (owner.pos - waypath.newway).sqnorm() <= WAYPOINT_TOLERANCE_SQ:
        waypath.advance()
        return ZERO_VECTOR
    else:
        return force_seek(owner, target)

def activate_waypathresume(steering, target):
    """Activate WAYPATHRESUME behaviour."""
    # TODO: Error checking here.
    if len(target) > 1:
        invk = 1.0/target[1]
    else:
        invk = 1.0/PATHRESUME_DECAY
    steering.targets['WAYPATHRESUME'] = (target[0], invk)
    return True

def force_flowfollow(owner, vel_field, dt=1.0):
    """Steering force for FLOWFOLLOW behaviour.

    Parameters
    ----------
    owner: SimpleVehicle2d
        The vehicle computing this force.
    vel_field: function Point2d(Point2d)
        A velocity vector field; owner will attempt to follow this.
    dt: Non-negative float
        Time between steering updates.
    """
    new_pos = owner.pos + owner.vel.scm(dt)
    target_vel = vel_field(new_pos)
    return (target_vel - owner.vel)

def activate_flowfollow(steering, target):
    """Activate FLOWFOLLOW behaviour."""
    # TODO: Error checking here.
    # owner_field = lambda pos: vel_field(pos).scm(vel_scale)
    steering.targets['FLOWFOLLOW'] = target
    return True

##############################################
### Group (flocking) behaviours start here ###
##############################################

def force_separate(owner):
    """Steering force for SEPARATE group behaviour (flocking).

    Parameters
    ----------
    owner: SimpleVehicle2d
        The vehicle computing this force.

    Notes
    -----
    Flocking forces use owner.neighbor_list to decide which vehicles to flock
    with; update this list before calling this function.

    For each neighbor, include a force away from that neighbor with magnitude
    proportional to the neighbor radius and inversely proprotional to distance.
    This gave nicer results and allows us to cleverly avoid computing a sqrt.
    """
    result = Point2d(0,0)
    for other in owner.neighbor_list:
        if other is not owner:
            offset = owner.pos - other.pos
            result += offset.scm(FLOCKING_SEPARATE_SCALE*other.radius/offset.sqnorm())
    return result

def activate_separate(steering, n_list):
    """Activate SEPARATE behaviour."""
    steering.flocking = True
    # TODO: Check for errors
    steering.targets['SEPARATE'] = ()
    steering.flockmates = n_list[:]
    return True

def force_align(owner):
    """Steering force for ALIGN group behaviour (flocking).

    Parameters
    ----------
    owner: SimpleVehicle2d
        The vehicle computing this force.

    Notes
    -----
    Flocking forces use owner.neighbor_list to decide which vehicles to flock
    with; update this list before calling this function.

    Unlike(?) traditional boids, we ALIGN with the average of neighbors'
    velocity vectors. Align with heading (normalize velocity) looked weird.
    """
    result = Point2d(0,0)
    n = 0
    for other in owner.neighbor_list:
        if other is not owner:
            result += other.vel
            n += 1
    if n > 0:
        result = result.scm(1.0/n)
        result -= owner.front
    return result

def activate_align(steering, n_list):
    """Activate ALIGN behaviour."""
    steering.flocking = True
    # TODO: Check for errors
    steering.targets['ALIGN'] = ()
    steering.flockmates = n_list[:]
    return True

def force_cohesion(owner):
    """Steering force for COHESION group behaviour.

    Parameters
    ----------
    owner: SimpleVehicle2d
        The vehicle computing this force.

    Notes
    -----
    Flocking forces use owner.neighbor_list to decide which vehicles to flock
    with; update this list before calling this function.
    """

    center = Point2d(0,0)
    n = 0
    for other in owner.neighbor_list:
        if other is not owner:
            center += other.pos
            n += 1
    if n > 0:
        center = center.scm(1.0/n)
        return force_arrive(owner, center, FLOCKING_COHESHION_HESITANCE)
    else:
        return ZERO_VECTOR

def activate_cohesion(steering, n_list):
    """Activate COHESION behaviour."""
    steering.flocking = True
    # TODO: Check for errors
    steering.targets['COHESION'] = ()
    steering.flockmates = n_list[:]
    return True

FLOCKING_LIST = ['SEPARATE', 'ALIGN', 'COHESION']
"""Flocking behaviours need additional set/pause/resume/stop checking."""

########################################################
## Auto-generate a list of behaviours above, along with
## dictionaries to reference their force/activate fnc's.
## This allows us to easily add behaviours later; see
## the module docstring for instructions.
########################################################
BEHAVIOUR_LIST = [x[6:].upper() for x in locals().keys() if x[:6] == 'force_']
FORCE_FNC = dict()
ACTIVATE_FNC = dict()
for behaviour in BEHAVIOUR_LIST[:]:
    try:
        force_fnc = locals()['force_' + behaviour.lower()]
        activate_fnc = locals()['activate_' + behaviour.lower()]
        FORCE_FNC[behaviour] = force_fnc
        ACTIVATE_FNC[behaviour] = activate_fnc
    except KeyError:
        logging.debug("[steering.py] Warning: could not define behaviour %s." % behaviour)
        BEHAVIOUR_LIST.remove(behaviour)

# Now make sure that expected flocking behaviours were correctly defined
for behaviour in FLOCKING_LIST:
    if not (behaviour in BEHAVIOUR_LIST):
        logging.debug("[steering.py] Warning: flocking %s is not available." % behaviour)
        FLOCKING_LIST.remove(behaviour)

########################################################
### Navigator-type class to control vehicle steering ###
########################################################

class SteeringBehavior(object):
    """Helper class for managing a vehicle's autonomous steering.

    Each vehicle should maintain a reference to an instance of this class,
    and call the compute_force() method when an update is needed.

    Parameters
    ----------
    vehicle: SimpleVehicle2d
        The vehicle that will be steered by this instance
    use_budget: boolean
        Default (True) uses vehicle's maxforce as a force budget per update.
        If set to False, all active behaviors are consdidered each update.
    """

    PRIORITY_DEFAULTS = ['BRAKE',
                         'WALLAVOID',
                         'AVOID',
                         'SEPARATE',
                         'FLEE',
                         'EVADE',
                         'SEEK',
                         'ARRIVE',
                         'TAKECOVER',
                         'PURSUE',
                         'GUARD',
                         'FOLLOW',
                         'WAYPATHRESUME',
                         'WAYPATHTRAVERSE',
                         'COHESION',
                         'ALIGN',
                         'FLOWFOLLOW',
                         'WANDER'
                        ]

    def __init__(self, vehicle, use_budget=True):
        self.vehicle = vehicle
        self.status = {beh: False for beh in BEHAVIOUR_LIST}
        self.targets = dict()
        self.inactive_targets = dict()
        self.flockmates = []
        self.flocking = False
        self.steering_force = Point2d(0,0)

        # Set the appropriate compute_force_ function here.
        if use_budget is True:
            self.compute_force = self.compute_force_budgeted
            # Unless this is overridden, sort behaviours by order in PRIORITY_DEFAULTS
            self.priority_order = SteeringBehavior.PRIORITY_DEFAULTS
            self.priority_key = lambda x: self.priority_order.index(x[0])
            self.set_priorities()
        else:
            self.compute_force = self.compute_force_simple

    def set_target(self, **kwargs):
        """Initializes one or more steering behaviours.

        Parameters
        ----------
        SEEK: (float, float), optional
            If given, the vehicle will begin SEEKing towards this point.
        FLEE: (float, float), optional
            If given, the vehicle will begin FLEEing towards this point.
        ARRIVE: (float, float), optional
            If given, the vehicle will begin ARRIVEing towards this point.
        PURSUE: BasePointMass2d, optional
            If given, the vehicle will begin PURSUEing the prey.
        EVADE: BasePointMass2d, optional
            If given, the vehicle will begin EVADEing the predator
        TAKECOVER: BasePointMass2d, optional
            If given, the vehicle will try to TAKECOVER from the predator.
        WANDER: tuple of int or float, optional
            (Distance, Radius, Jitter) for WANDER behaviour
        AVOID: tuple of BasePointMass2d, optional
            Tuple (iterable ok?) of obstacles to be avoided.
        WALLAVOID: tuple of BaseWall2d, optional
            List of walls to be avoided
        GUARD: (BasePointMass2d, BasePointMass2d, float), optional
            (GuardTarget, GuardFrom, AggressivePercent)
        WAYPATHTRAVERSE: (WaypointPath), optional
            List of waypoints for WAYPATHTRAVERSE behaviour.
        WAYPATHRESUME: (WaypointPath, invk), optional
            List of waypoints and inverse of decay constant for PATHRESUME.
        FLOWFOLLOW: (vel_field, dt), optional
            Callable vel_field function and time increment
        FOLLOW: (BasePointMass2d, Point2d), optional
            (Leader, OffsetFromLeader)
        SEPARATE: List of BasePointMass2d, optional
            List of targets to flock with
        ALIGN: List of BasePointMass2d, optional
            List of targets to flock with
        COHESION: List of BasePointMass2d, optional
            List of targets to flock with
        BRAKE: float, optional
            Speed decay factor (0 < decay < 1)

        Returns
        -------
        list of boolean (or single boolean)
            Each item True/False according to whether initialization succeeded.

        Notes
        -----
        Flocking behaviours (SEPARATE, ALIGN, COHESION) automatically set
        self.flocking to True; this is used by force_foo functions so that
        neighbors need only be tagged once per cycle (for efficiency).
        """
        all_res = []
        for (behaviour, target) in kwargs.items():
            # Find and call correponding activate function
            try:
                # The activate_foo function must be defined above
                activate = ACTIVATE_FNC[behaviour]
                result = activate(self, target)
                if result is True:
                    self.status[behaviour] = True
                    logging.debug('%s successfully initiated.' % behaviour)
                    all_res.append(True)
            except KeyError:
                logging.debug("Warning: %s behaviour improperly defined; cannot activate." % behaviour)
                all_res.append(False)
        self.set_priorities()
        # If we only initialized one behaviour, don't return a list.
        if len(all_res) == 1:
            return all_res[0]
        else:
            return all_res

    def pause(self, steering_type):
        """Temporarilily turns off a steering behaviour, storing targets for later.

        Parameters
        ----------
        steering_type: string
            Name of the behaviour to be paused.

        Returns
        -------
        boolean
            True if the pause was successful, False otherwise.
        """
        # If behaviour has not been properly activated, warn and exit.
        try:
            self.inactive_targets[steering_type] = self.targets[steering_type]
        except KeyError:
            logging.debug('Warning: Behaviour %s has not been initialized. Ignoring pause.' % steering_type)
            return False
        # Otherwise, pause until later resumed.
        del self.targets[steering_type]
        self.status[steering_type] = False
        self.set_priorities()
        logging.debug('%s paused.' % steering_type)
        return True

    def resume(self, steering_type):
        """Turns on a previously paused behaviour, using old targets.

        Parameters
        ----------
        steering_type: string
            Name of the behaviour to be resumed.

        Returns
        -------
        boolean
            True if the resume was successful, False otherwise.
        """
        # If behaviour was not previously paused, warn and exit.
        try:
            target = self.inactive_targets[steering_type]
        except KeyError:
            logging.debug('Warning: Behaviour %s was not paused. Ignoring resume.' % steering_type)
            return False
        # Otherwise, retreive previously-saved targets and resume.
        self.targets[steering_type] = target
        del self.inactive_targets[steering_type]
        self.status[steering_type] = True
        self.set_priorities()
        logging.debug('%s resumed.' % steering_type)
        return True

    def stop(self, steering_type):
        """Permanently turns off a steering behaviour until re-initialized.

        Parameters
        ----------
        steering_type: string
            Name of the behaviour to be stopped.

        Returns
        -------
        boolean
            True if the stop was successful, False otherwise.
        """
        # If behaviour has not been properly activated, warn and exit.
        try:
            del self.targets[steering_type]
        except KeyError:
            logging.debug('Warning: Behaviour %s has not been initialized. Ignoring stop.' % steering_type)
            return False
        # Otherwise, stop this behaviour (without storing prior targets)
        self.status[steering_type] = False
        self.set_priorities()
        logging.debug('%s stopped.' % steering_type)

    def update_flocking_status(self):
        """Sets or clears flocking status based on currently-active behaviours."""
        flock_yes = False
        for steering_type in FLOCKING_LIST:
            if self.status[steering_type] is True:
                flock_yes = True
                break
        self.flocking = flock_yes

    def flag_neighbor_vehicles(self, vehlist=[]):
        """Populates a list of nearby vehicles, for use with flocking.

        Parameters
        ----------
        vehlist: List of BasePointMass2d
            List of vehicles to be checked against. See Notes below.

        Notes
        -----
        This function checks other vehicles based on their distance to owner and
        includes only vehicles in front of the owner. Maximum distance is the
        owner's radius times FLOCKING_RADIUS_MULTIPLIER. We may consider more
        sophisticated sensing of neighbors in the future.

        Any pre-processing (such as spatial partitioning, sensory perception, or
        flocking with certain vehicles only) should be done before calling this
        function; with those results passed in as vehlist.

        Results of flagging are stored as owner.neighbor_list to be read later by
        force_foo functions (mostly flocking) that require neighbor information.
        Run this function before any such force_foo functions).
        """
        owner = self.vehicle
        n_radius = owner.radius * FLOCKING_RADIUS_MULTIPLIER
        neighbor_list = list()
        for other in vehlist:
            if other is not owner:
                min_range = n_radius + other.radius
                offset = other.pos - owner.pos
                if offset.sqnorm() < min_range * min_range:
                    # Only consider neighbors to the front
                    if offset*owner.front >= 0:
                        neighbor_list.append(other)
        owner.neighbor_list = neighbor_list

    def compute_force_simple(self):
        """Compute steering force using all currently-active behaviors.

        Returns
        -------
        Point2d: Steering force.

        Note
        ----
        This considers all active behaviours, but will still limit the final
        force vector's magnitude to the owner's maxforce.
        """
        self.steering_force.zero()
        owner = self.vehicle
        # If any flocking is active, determine neighbors first
        if self.flocking is True:
            self.flag_neighbor_vehicles(self.flockmates)
        # Iterate over active behaviours and accumulate force from each
        for (behaviour, targets) in self.targets.iteritems():
            self.steering_force += FORCE_FNC[behaviour](owner, *targets)
        self.steering_force.truncate(owner.maxforce)
        return self.steering_force

    def set_priorities(self):
        """Create a prioritized list of steering behaviours for later use."""
        pkey = lambda x: self.priority_order.index(x[0])
        self.priorities = sorted(self.targets.items(), key=pkey)
        self.update_flocking_status()

    def compute_force_budgeted(self):
        """Find prioritized steering force within the vehicle's budget.

        Returns
        -------
        Point2d: Steering force.
        """
        self.steering_force.zero()
        owner = self.vehicle
        # If any flocking is active, determine neighbors first
        if self.flocking is True:
            self.flag_neighbor_vehicles(self.flockmates)

        budget = owner.maxforce
        for (behaviour, targets) in self.priorities:
            # Check if this behaviour is actually active
            if self.status[behaviour] is not True:
                continue
            # If so, call the behaviour's force_ function
            newforce = FORCE_FNC[behaviour](owner, *targets)
            newnorm = newforce.norm()
            if budget > newnorm:
                # If there is enough force budget left, continue as usual
                self.steering_force += newforce
                budget -= newnorm
            else:
                # Scale newforce to remaining budget, apply, and exit
                newforce.scm(budget/newnorm)
                self.steering_force += newforce
                return self.steering_force

        # If any budget is leftover, just return the total force
        return self.steering_force

if __name__ == "__main__":
    print("Steering behavior functions. Import this elsewhere. Implemented behaviours are:")
    print(sorted(BEHAVIOUR_LIST))
    print("\nAvailable flocking behaviours are:")
    print(sorted(FLOCKING_LIST))
    print("\nValues of imported steering constants are:")
    print(STEERING_DEFAULTS)