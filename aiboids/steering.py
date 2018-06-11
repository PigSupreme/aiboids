# steering.py
"""Module containing AiBoid steering behavior functions (and Navigator).

Many behaviours use constant values, imported from steering_constants.py and
assigned to local constants within this module. These are chosen based on the
the physics defaults (also in steering_constants) to give reasonable results,
but can be overridden with steering.FOO_CONSTANT = new_value. [check this]

Todo:
    Flocking behaviours all assume the flock_list remains constant. If we don't
    want this, it may be better to have store flock_list on the owner and have
    it managed externally (perhaps via the owner's Navigator object).

    Another side effect is that each instance of flocking stores its own copy
    of the flock_list, and this isn't very efficient use of storage.

Todo:
    Fix the rest of the module docstring below, once we've finished changes.



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

import logging

from sys import path
path.append('..')
from aiboids.point2d import Point2d

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

from abc import ABCMeta, abstractmethod
from collections import OrderedDict

# Dictionary of default constants for steering behaviours
from aiboids.steering_constants import STEERING_DEFAULTS
for const_name, const_value in STEERING_DEFAULTS.items():
    vars()[const_name] = const_value

# Order in which behaviours are considered when using budgeted force:
from aiboids.steering_constants import PRIORITY_DEFAULTS
PRIORITY_KEY = lambda x: PRIORITY_DEFAULTS.index(x[0])

class SteeringBehaviour(object):
    """Abstract Base Class for all steering behaviours.

    Args:
        owner (vehicle): Steering is computed for this vehicle.

    Subclass __init__() methods should first call::

            SteeringBehaviour.__init__(self, owner)

    Where owner is the vehicle that will use the behaviour. This sets the owner
    locally (and may do other things in the future).

    Subclasses must implement the force() method with *self* as the only
    argument; this is used by the Navigator class for updates.

    See the Seek class for a simple example.
    """
    __metaclass__ = ABCMeta
    def __init__(self, owner):
        self.owner = owner

    @abstractmethod
    def force(self):
        """Compute the owner's steering force for this behaviour."""
        pass


class Seek(SteeringBehaviour):
    """SEEK towards a fixed point at maximum speed.

    Args:
        owner (SimpleVehicle2d): The vehicle computing this force.
        target (Point2d): The point to SEEK towards.

    Note:
        Use ARRIVE for a more graceful approach and to prevent jittering.
    """
    def __init__(self, owner, target):
        SteeringBehaviour.__init__(self, owner)
        self.target = target

    def force(self):
        owner = self.owner
        targetvel = (self.target - owner.pos)
        targetvel.scale_to(owner.maxspeed)
        return targetvel - owner.vel


class Flee(SteeringBehaviour):
    """FLEE from a fixed point at maximum speed.

    Args:
        owner (SimpleVehicle2d): The vehicle computing this force.
        target (Point2d): The point to FLEE from.
        panic_dist (float, optional): If specified, FLEE only when the
            distance to the target is less than this value.
    """
    def __init__(self, owner, target, panic_dist=INF):
        SteeringBehaviour.__init__(self, owner)
        self.target = target
        self.panic_sq = panic_dist**2

    def force(self):
        owner = self.owner
        targetvel = (owner.pos - self.target)
        if 0 < targetvel.sqnorm() < self.panic_sq:
            targetvel = (owner.maxspeed)*targetvel.unit()
            return targetvel - owner.vel
        else:
            return ZERO_VECTOR


class Arrive(SteeringBehaviour):
    """Gracefully ARRIVE at a target point.

    Args:
        owner (SimpleVehicle2d): The vehicle computing this force.
        target (Point2d): The target point that owner is to arrive at.
        hesistance (float): Controls the time it takes to deccelerate;
            higher values give more gradual (and slow) decceleration.
            Suggested values are 1.0 - 10.0; default is 2.0.

    This works like SEEK, except the vehicle gradually deccelerates as it
    nears the target position.

    Todo:
        Stability analysis suggested that hesistance should be set above a
        ceratin threshold, based on the owner's mass. Implement this as
        the default value and/or scale based on this threshold.
    """
    def __init__(self, owner, target, hesitance=ARRIVE_DEFAULT_HESITANCE):
        SteeringBehaviour.__init__(self, owner)
        self.target = target
        self.hesitance = hesitance

    def force(self):
        owner = self.owner
        target_offset = (self.target - owner.pos)
        dist = target_offset.norm()
        if dist > 0:
            speed = dist / (ARRIVE_DECEL_TWEAK * self.hesitance)
            if speed > owner.maxspeed:
                speed = owner.maxspeed
            targetvel = (speed/dist)*target_offset
            return targetvel - owner.vel
        else:
            return ZERO_VECTOR


class Wander(SteeringBehaviour):
    """Pseudo-randomly WANDER about.

    Args:
        owner (SimpleVehicle2d): The vehicle computing this force.
        distance (float): Distance to center of WANDER circle; see below.
        radius (float): Radius of the WANDER circle; see below.
        jitter (float): Maximum jitter magnitude; see below.

    WANDER projects an imaginary circle in directly front of the owner and
    will SEEK towards a randomly-moving target on that circle. The center and
    radius of this circle are determined by *distance* and *radius*.

    We displace the target point each update by a random vector (whose size is
    limited by *jitter*) and rescale so the target remains on our circle.
    """
    def __init__(self, owner, distance=WANDER_DISTANCE, radius=WANDER_RADIUS, jitter=WANDER_JITTER):
        SteeringBehaviour.__init__(self, owner)
        self.radius = radius
        self.distance = distance
        self.jitter = jitter
        self.offset = radius*owner.front

    def force(self):
        # Add a random displacement to previous target and re-scale
        self.offset += Point2d(rand_uni(self.jitter), rand_uni(self.jitter))
        self.offset.scale_to(self.radius)
        # This SEEKs to the final target, using a computational trick
        targetvel = self.distance*self.owner.front + self.offset
        targetvel.scale_to(self.owner.maxspeed)
        return targetvel - self.owner.vel


class ObstacleAvoid(SteeringBehaviour):
    """AVOID stationary obstacles by steering around them.

    Args:
        owner (SimpleVehicle2d): The vehicle computing this force.
        obs_list (list, SimpleObstacle2d): Obstacles to check for avoidance.

    This projects a box in front of the owner and tries to find an obstacle
    for which collision is imminent (not always the closest obstacle). The
    owner will attempt to steer around that obstacle.
    """
    def __init__(self, owner, obstacle_list):
        SteeringBehaviour.__init__(self, owner)
        self.obstacles = tuple(obstacle_list)

    def force(self):
        owner = self.owner
        # Obstacles closer than this distance will be avoided
        front_d = (1 + owner.vel.norm()/owner.maxspeed)*OBSTACLEAVOID_MIN_LENGTH
        front_sq = front_d * front_d

        # Find the closest obstacle within the detection box
        xmin = 1 + front_d
        obs_closest = None
        for obstacle in self.obstacles:
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
                    radicand = expr**2 - local_y**2
                    if radicand > 0:
                        xval = local_x - sqrt(expr*expr - local_y*local_y)
                        # If this obstacle is closer, update minimum values
                        if xval < xmin:
                            xmin, lx, ly = xval, local_x, local_y
                            obs_closest = obstacle

        # If there is a closest obstacle, avoid it
        if obs_closest:
            lr = obs_closest.radius + owner.radius
            if ly >= 0:
                lat_scale = (ly - lr)*(2.0 - lr / front_d)
            else:
                lat_scale = (ly + lr)*(2.0 - lr / front_d)
            brake_scale = (lr - lx)*OBSTACLEAVOID_BRAKE_WEIGHT
            result = (brake_scale)*owner.front + (lat_scale)*owner.left
            return result
        else:
            return ZERO_VECTOR


class TakeCover(SteeringBehaviour):
    """TAKECOVER from another target vehicle behind a nearby obstacle.

    Args:
        owner (SimpleVehicle2d): The vehicle computing this force.
        target (BasePointMass2d): The vehicle we try to hide from.
        obs_list (list of BasePointMass2d): List of obstacles for hiding.
        max_range (float): Hiding points further than this value are ignored.
        stalk (boolean, optional): If True, only hide when we are in front of the target.

    Owner attempts to find a position that will put an obstacle between itself
    and some other vehicle. If such a point cannot be found nearby (within
    max_range of the owner), EVADE the target predator instead.

    If stalk is set to True, the owner will try to hide only when in front of
    the target and within a certain range. Stalking should be used with other
    behaviours (e.g., PURSUE) or we get very odd-looking results.

    Todo:
        Consider moving stalk to a seperate behaviour.
    """
    def __init__(self, owner, target, obstacle_list, max_range, stalk=False):
        SteeringBehaviour.__init__(self, owner)
        self.target = target
        self.obstacles = tuple(obstacle_list)
        self.range_sq = max_range**2
        self.stalk = stalk
        # Note: This helper instance of EVADE avoids duplicating code here.
        self.evade_helper = Evade(owner, target, TAKECOVER_EVADE_MULT*max_range)
        # TODO: ARRIVE needs a fixed target, so it's more trouble than it's
        # worth to use a helper instance here. If we ever provide an easy way
        # to switch behaviour targets, it might be worth updating this code and
        # replace the duplicated ARRIVE code in the force() method.
        self.hesitance = TAKECOVER_ARRIVE_HESITANCE

    def force(self):
        # If stalking, exit unless we're close in front of our target.
        if self.stalk:
            hide_dir = (self.owner.pos - self.target.pos)
            if hide_dir.sqnorm() < TAKECOVER_STALK_DSQ:
                if (hide_dir.unit() * self.target.front) < TAKECOVER_STALK_COS:
                    return ZERO_VECTOR
        # Otherwise, find the nearest hiding spot.
        best_dsq = self.range_sq
        best_pos = None
        for obs in self.obstacles:
            # Hiding point for this obstacle, is directly opposite the
            # vehicle we're hiding from.
            hide_dir = (obs.pos - self.target.pos).unit()
            hide_pos = obs.pos + (obs.radius + TAKECOVER_OBSTACLE_PROXIMITY*self.owner.radius)*hide_dir
            hide_dsq = (hide_pos - self.owner.pos).sqnorm()
            if hide_dsq < best_dsq:
                best_pos = hide_pos
                best_dsq = hide_dsq
        # If no nearby hiding spot was found, just EVADE the target.
        if best_pos is None:
            return self.evade_helper.force()
        # Otherwise, ARRIVE at the hiding spot.
        else:
            target_offset = (best_pos - self.owner.pos)
            dist = target_offset.norm()
            if dist > 0:
                speed = dist / (ARRIVE_DECEL_TWEAK * self.hesitance)
                if speed > self.owner.maxspeed:
                    speed = self.owner.maxspeed
                targetvel = (speed/dist)*target_offset
                return targetvel - self.owner.vel
            else:
                return ZERO_VECTOR


class WallAvoid(SteeringBehaviour):
    """WALLAVOID behaviour with three whiskers.

    Args:
        owner (SimpleVehicle2d): The vehicle computing this force.
        front_length (float): Length of the forward whisker.
        wall_list (list, BaseWall2d): List of walls to test against.

    This uses a virtual whisker in front of the vehicle, and two side
    whiskers at 45 degrees from the front. The sides are slighly smaller;
    length is scaled by the WALLAVOID_SIDE_SCALE steering constant.

    For each whisker, we find the wall having its point of intersection
    closest to the base of the whisker. If such a wall is detected, it
    contributes a force in the direction of the wall normal, proportional
    to the penetration depth of the whisker.
    """
    def __init__(self, owner, front_length, wall_list):
        SteeringBehaviour.__init__(self, owner)
        self.whisker_coords = ((1,0), (SQRT_HALF,SQRT_HALF), (SQRT_HALF,-SQRT_HALF))
        side_length = front_length * WALLAVOID_SIDE_SCALE
        self.whisker_sizes = (front_length, side_length, side_length)
        self.whisker_num = 3
        self.walls = wall_list

    def force(self):
        owner = self.owner
        whisker_tip = self.whisker_num*[0]
        closest_wall = self.whisker_num*[None]

        # Convert unit vectors for each whisker to global coordinates
        for i in range(self.whisker_num):
            (u,v) = self.whisker_coords[i]
            whisker_tip[i] = u*owner.front + v*owner.left

        # This will hold the shortest distances along each whisker to a wall,
        # but ignoring any walls beyond the length of the whisker.
        t_min = list(self.whisker_sizes)

        # Find the closest wall intersecting each whisker
        for wall in self.walls:
            # TODO: Document the vector math that makes this magic work!
            # Numerator of intersection test is the same for all whiskers
            t_numer = wall.front * (wall.pos - owner.pos)
            for i in range(self.whisker_num):
                # Is vehicle in front and whisker tip behind wall's infinite line?
                try:
                    t = t_numer / (wall.front * whisker_tip[i])
                except ZeroDivisionError:
                    # Whisker is parallel to wall in this case, no intersection
                    continue
                if 0 < t < t_min[i]:
                    # Is the point of intersection actually on the wall segment?
                    poi = owner.pos + t*whisker_tip[i]
                    if (wall.pos - poi).sqnorm() <= wall.rsq:
                        # This is the closest intersecting wall so far
                        closest_wall[i] = wall
                        t_min[i] = t

        # For each whisker, add the force away from the closest wall (if any)
        result = Point2d(0,0)
        for i in range(self.whisker_num):
            if closest_wall[i] is not None:
                depth = self.whisker_sizes[i] - t_min[i]
                result += depth*closest_wall[i].front

        # Scale by owner radius; bigger objects should tend to stay away
        return owner.radius*result


class Pursue(SteeringBehaviour):
    """PURSUE a moving object.

    Args:
        owner (SimpleVehicle2d): The vehicle computing this force.
        prey (BasePointMass2d): The object we're pursuing.
        pcos (float, optional): Cosine of the pounce angle, see notes.
        pdist (float, optional): Pounce distance, see notes.

    Notes:
        If the prey is heading our way (we are within a certain angle of
        the prey's heading) and within a certain distance, simply "pounce"
        on the prey by SEEKing to its current position. Otherwise, predict
        the future position of they prey, based on current velocities,
        and SEEK to that location.
    """
    def __init__(self, owner, prey, pcos=PURSUE_POUNCE_COS, pdist=PURSUE_POUNCE_DISTANCE):
        SteeringBehaviour.__init__(self, owner)

        self.prey = prey
        self.pcos = pcos
        self.pdist_sq = pdist**2

    def force(self):
        owner = self.owner
        prey = self.prey
        offset = prey.pos - owner.pos
        dsq = offset.sqnorm()
        # If prey is close and COMING RIGHT FOR US!!!, target prey's position.
        # The angle computation assumes prey.front is a unit vector.
        if dsq < self.pdist_sq and offset * prey.front < -self.pcos * sqrt(dsq):
            target = prey.pos
            targetvel = (target - owner.pos)
        # Otherwise, predict the future position of prey, assuming it will keep
        # a constant velocity.
        else:
            ptime = sqrt(dsq)/(owner.maxspeed + prey.vel.norm())
            target = ptime * prey.vel + prey.pos
        # Now SEEK to the target position
        targetvel = (target - owner.pos)
        targetvel.scale_to(owner.maxspeed)
        return targetvel - owner.vel


class Follow(SteeringBehaviour):
    """Attempt to FOLLOW a leader at some fixed offset.

    Args:
        owner (SimpleVehicle2d): The vehicle computing this force.
        leader (BasePointMass2d): The object to be followed.
        offset (Point2d):  Offset from leader. Given in the leader's local
            coordinate frame, with front = +x.
    """
    def __init__(self, owner, leader, offset, hesitance=FOLLOW_ARRIVE_HESITANCE):
        SteeringBehaviour.__init__(self, owner)
        self.leader = leader
        self.offset = offset
        self.hesitance = hesitance

    def force(self):
        owner = self.owner
        leader = self.leader
        offset = self.offset
        target_pos = leader.pos + (offset[0])*leader.front + (offset[1])*leader.left
        diff = target_pos - self.owner.pos
        ptime = diff.norm() / (owner.maxspeed + leader.vel.norm())
        target_pos += ptime* leader.vel

        # Now ARRIVE at the target position
        offset = (target_pos - owner.pos)
        dist = offset.norm()
        if dist > 0:
            speed = dist / (ARRIVE_DECEL_TWEAK * self.hesitance)
            if speed > owner.maxspeed:
                speed = owner.maxspeed
            targetvel = (speed/dist)*offset
            return targetvel - owner.vel
        else:
            return ZERO_VECTOR


class Evade(SteeringBehaviour):
    """EVADE a moving object.

        Args:
            owner (SimpleVehicle2d): The vehicle computing this force.
            predator (BasePointMass2d): The object we're evading.
            panic_dist (float): Ignore the predator beyond this distance.

        Notes:
            If the prey is heading our way (we are within a certain angle of
            the prey's heading) and within a certain distance, simply "pounce"
            on the prey by SEEKing to its current position. Otherwise, predict
            the future position of they prey, based on current velocities,
            and SEEK to that location.
    """
    def __init__(self, owner, predator, panic_dist=EVADE_PANIC_DIST):

        SteeringBehaviour.__init__(self, owner)
        self.predator = predator
        self.panic_sq = panic_dist**2

    def force(self):
        owner = self.owner
        predator = self.predator
        offset = predator.pos - owner.pos
        # Ignore the predator if it's too far away
        if offset.sqnorm() >= self.panic_sq:
            return ZERO_VECTOR
        # Otherwise, predict the future position of predator, assuming it will
        # keep a constant velocity. Just like PURSUE (without the pounce).
        ptime = offset.norm()/(owner.maxspeed + predator.vel.norm())
        # Now FLEE from the target position
        target = ptime * predator.vel + predator.pos
        targetvel = (owner.pos - target)
        targetvel.scale_to(owner.maxspeed)
        return targetvel - owner.vel


class Guard(SteeringBehaviour):
    """GUARD one object from another by moving between them.

    Args:
        owner (SimpleVehicle2d): The vehicle computing this force.
        guard_this (BasePointMass2d): The object to be guarded.
        guard_from (BasePointMass2d): The object to guard against.
        aggro (float): Value from 0 to 1 for aggressiveness; see below.

    This is a more general version of INTERPOSE. The vehicle attempts to
    keep itself between *guard_this* and *guard_from*, at a relative
    distance controlled by *aggro*. An *aggro* near zero will position
    close to *guard_this*; near 1.0 will position close to *guard_from*.

    Note:
        This allows setting *aggro* outside of the interval [0,1]; as the
        formula for computing position is the standard parameterization of
        the line segment from *guard_this* to *guard_from*.
    """
    def __init__(self, owner, guard_this, guard_from, aggro=0.5):

        SteeringBehaviour.__init__(self, owner)
        self.guard_this = guard_this
        self.guard_from = guard_from
        self.aggro = aggro

    def force(self):
        # Find the desired position between the two objects right now
        owner = self.owner
        this_pos = self.guard_this.pos
        from_pos = self.guard_from.pos
        want_pos = this_pos + self.aggro*(from_pos - this_pos)

        # Predict future positions based on owner's time to that position
        est_time = (want_pos - owner.pos).norm()/owner.maxspeed
        this_pos += est_time * self.guard_this.vel
        from_pos += est_time * self.guard_from.vel

        # Now ARRIVE at a spot between the predicted positions
        want_pos = this_pos + self.aggro* (from_pos - this_pos)
        offset = (want_pos - owner.pos)
        dist = offset.norm()
        if dist > 0:
            speed = dist / (ARRIVE_DECEL_TWEAK * GUARD_HESITANCE)
            if speed > owner.maxspeed:
                speed = owner.maxspeed
            targetvel = (speed/dist)*offset
            return targetvel - owner.vel
        else:
            return ZERO_VECTOR


class Brake(SteeringBehaviour):
    """Steering force opposite of current forward velocity.

    Args:
        owner (SimpleVehicle2d): The vehicle computing this force.
        decay (float): Discrete exponential decay constant; 0 < decay < 1.

    Warning:
        Haven't sufficiently tested this. Values of decay close to 0 should
        give more gradual braking; close to 1 should be more severe. The
        actual performance may depend on delta_t for time.
    """
    def __init__(self, owner, decay=0.5):
        SteeringBehaviour.__init__(self, owner)
        self.decay = -decay

    def force(self):
        return self.decay * self.owner.vel


##############################################
### Path-related behaviours start here     ###
##############################################
class WaypointPath(object):
    """Helper class for managing path-related behaviour, using waypoints.

    Args:
        waypoints (list of Point2d): Non-empty list of waypoints on this path.
        is_cyclic (boolean): If set to True, path will automatically cycle. See notes.

    Notes:
        Instances of WaypointPath should be owned by a Navigator instance, but
        all path-management code is controlled from within this class.

        When using this for vehicle steering, the first waypoint is intended as
        the starting point of some owner vehicle. The vehicle will *not*
        automatically return to this point even if is_cyclic is set to True, so
        add it manually to the end of *waypoints* if a return trip is needed.

    Todo:
        It may be helpful to rewrite this class as a generator.
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
        self.edgevector = (1.0/self.edgelength)*offset

        self.is_cyclic = is_cyclic

    def reset_from_position(self, start_pos, do_return=False):
        """Reset the next waypoint to the start of this path.

        Args:
            start_pos (Point2d): The new starting point for the path
            do_return (boolean, optional): If set to True, start_pos becomes
                the final waypoint. See Notes.

        Notes:
            As with the __init__() method, start_pos is intended as the current
            location of some vehicle, and is not explicitly added as the first
            waypoint. If the path was previously cyclic, we will not return to
            start_pos by default (but the previous waypoints will still continue
            to cycle). To override this, set do_return=True. However, start_pos
            will not be added explicitly is it is within the threshold given by
            PATH_EPSILON_SQ, because it is close enough to an actual waypoint.

        Todo:
            Make sure this works for single edges and start_pos close to the
            the first or last waypoint in a cyclic path.
        """
        self.newway = self.waypoints[0]
        self.wpindex = 0
        # If we're close to the first waypoint, use that wp as start_pos
        if (start_pos - self.newway).sqnorm() < PATH_EPSILON_SQ:
            self.advance()
        if do_return and (start_pos - self.waypoints[-1]).sqnorm() >= PATH_EPSILON_SQ:
            self.waypoints.append(start_pos)

    def advance(self):
        """Update our waypoint to the next one in the path.

        Note:
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
            self.edgevector = (1.0/self.edgelength)*offset

        # This throws if we are at the last waypoint in the list.
        except IndexError:
            if self.is_cyclic:
                # If cyclic, go back to the first waypoint
                self.wpindex = 0
                self.newway = self.waypoints[0]
                offset = self.newway - self.oldway
                self.edgelength = offset.norm()
                self.edgevector = (1.0/self.edgelength)*offset
            else:
                self.newway = None
                self.edgelength = 0
                self.edgevector = None

    def num_left(self):
        """Returns the number of waypoints remaining in this path.

        Note:
            For cyclic paths, we always return the total number of waypoints,
            regardless of where we are in the list.
        """
        if self.is_cyclic:
            return len(self.waypoints)
        else:
            return len(self.waypoints) - self.wpindex


class WaypathVisit(SteeringBehaviour):
    """Visit a series of waypoints in order.

    Args:
        owner (SimpleVehicle2d): The vehicle computing this force.
        waypath (WaypointPath): The path to be followed.
        wayradius (float): Waypoint radius; see notes.

    Note:
        In this version; we merely head towards the next waypoint. If there is
        only one waypoint left, we ARRIVE at it. Otherwise, we SEEK. A waypoint
        is visited once the distance to owner is less than *wayradius*.

    Todo:
        This is less of a behaviour and more of a manager for other behaviours;
        consider moving its functionality to within the Navigator class.
    """
    def __init__(self, owner, waypath, wayradius=WAYPOINT_RADIUS):
        SteeringBehaviour.__init__(self, owner)
        # TODO: If waypath has no waypoints left, ARRIVE won't work?
        self.waypath = waypath
        self.wprad_sq = wayradius**2
        if waypath.num_left() <= 1:
            self.wayforce = Arrive(owner, waypath.newway)
        else:
            self.wayforce = Seek(owner, waypath.newway)

    def force(self):
        # If no waypoints remain in the path, exit and return zero force.
        if self.waypath.newway is None:
            return ZERO_VECTOR
        # Otherwise, check if we're within distance the next waypoint.
        # Note: No force is returned if we switch to the next waypoint.
        if (self.owner.pos - self.waypath.newway).sqnorm() <= self.wprad_sq:
            self.waypath.advance()
            nextwaypt = self.waypath.newway
            # If this was the last waypoint in the path, we should already be
            # using ARRIVE, and don't need to change anything. Otherwise we
            # update the steering behaviour to the new waypoint.
            if nextwaypt is not None:
                # TODO: Next line used only by the demo for testing??
                self.owner.waypoint = nextwaypt
                del self.wayforce
                if self.waypath.num_left() <= 1:
                    self.wayforce = Arrive(self.owner, nextwaypt)
                else:
                    self.wayforce = Seek(self.owner, nextwaypt)
            return ZERO_VECTOR
        # Otherwise, we're still in progress to the current waypoint.
        return self.wayforce.force()


class WaypathResume(SteeringBehaviour):
    """Visit waypoints in order, trying to stay close to the path between them.

    Args:
        owner (SimpleVehicle2d): The vehicle computing this force.
        waypath (WaypointPath): The path to be followed.
        expk (positive float): Exponential decay constant; see notes.
        wayradius (float): Waypoint radius; see notes.

    Note:
        If the vehicle is knocked off-course, this will give a balance between
        returning directly to the current path edge and progressing towards the
        next waypoint; this is controlled by *expk*; larger values give a more
        immediate return to the path.

        If the vehicle has already overshot the next waypoint, we head directly
        to that waypoint, ignoring the path. Otherwise, follow an exponential
        decay curve asymptotic to the path; although this curve doesn't truly
        pass through the waypoint, it makes computations very quick, especially
        since we store invk. As above, a waypoint is visited once the distance
        to owner is less than *wayradius*, so we don't need to hit the waypoint
        exactly.

    Todo:
        This is less of a behaviour and more of a manager for other behaviours;
        consider moving its functionality to within the Navigator class.
    """
    def __init__(self, owner, waypath, expk=PATHRESUME_DECAY, wayradius=WAYPOINT_RADIUS):
        SteeringBehaviour.__init__(self, owner)
        self.waypath = waypath
        self.invk = 1.0/expk
        self.wprad_sq = wayradius**2

    def force(self):
        owner = self.owner
        nextwaypt = self.waypath.newway
        # If no waypoint left, exit immediately
        if nextwaypt is None:
            return ZERO_VECTOR

        # This is the remaining direct distance to the next waypoint,
        # using orthogonal projection operator.
        rl = (nextwaypt - owner.pos)/self.waypath.edgevector

        # We use an amazing computational shortcut to check if the resume point
        # (on the actual path) would be beyond the next waypoint. If so...
        if self.invk >= rl:
            # ...use ARRIVE if this is the last waypoint, and exit so that we
            # don't bother checking if we've reached it.
            if self.waypath.num_left() <= 1:
                target_offset = (nextwaypt - owner.pos)
                dist = target_offset.norm()
                if dist > 0:
                    speed = min(dist / ARRIVE_DECEL_TWEAK, owner.maxspeed)
                    targetvel = (speed/dist)*target_offset
                    return targetvel - owner.vel
                else:
                    return ZERO_VECTOR
            # ...otherwise, set the SEEK target for later computation.
            else:
                target = nextwaypt
        # Else, the resume point is still between last/next waypoints, and we
        # need to compute it (for later SEEKing)
        else:
            target = nextwaypt + (self.invk - rl)*self.waypath.edgevector

        # If we're still here, the above computations say we must SEEK to a
        # resume point or a waypointthat isn't the final one in the path. So...
        # Check if we're close enough to the next waypoint to switch; if so,
        # return zero force during this update.
        if (owner.pos - nextwaypt).sqnorm() <= self.wprad_sq:
            self.waypath.advance()
            return ZERO_VECTOR
        # Otherwise, SEEK to whatever target was computed above
        else:
            targetvel = (target - owner.pos)
            targetvel.scale_to(owner.maxspeed)
            return targetvel - owner.vel


class FlowFollow(SteeringBehaviour):
    """Steering force for FLOWFOLLOW behaviour.

    Args:
        owner (SimpleVehicle2d): The vehicle computing this force.
        vel_field (function Point2d(Point2d)):  A velocity vector field; owner
            will attempt to follow this.
        dt (positive float): Time between steering updates.

    Todo:
        Still experimental. We should probably allow for a variable time step.
    """
    def __init__(self, owner, vel_field, dt=1.0):
        SteeringBehaviour.__init__(self, owner)
        self.owner = owner
        self.flow = vel_field
        self.step = dt

    def force(self):
        new_pos = self.owner.pos + self.step*self.owner.vel
        # Computational shortcut, using desired velocity as the average of the
        # owner's current velocity and the predicted velocity.
        return 0.5*(self.flow(new_pos) - self.owner.vel)


##############################################
### Group (flocking) behaviours start here ###
##############################################
# TODO: When initiating flocking, check that owner has a flockmates attribute.

class FlockSeparate(SteeringBehaviour):
    """Steering force for SEPARATE group behaviour (flocking).

    Args:
        owner (SimpleVehicle2d): The vehicle computing this force.

    Notes:
        For each neighbor, include a force away from that neighbor of magnitude
        proportional to the neighbor radius and inversely proprotional to
        distance. This gave nicer results and allows us to cleverly avoid
        computing a sqrt.
    """
    def __init__(self, owner):
        SteeringBehaviour.__init__(self, owner)
        self.owner = owner
        owner.flocking = True

    def force(self):
        result = Point2d(0,0)
        for other in self.owner.neighbor_list:
            if other is not self.owner:
                offset = self.owner.pos - other.pos
                result += (FLOCKING_SEPARATE_SCALE*other.radius/offset.sqnorm())*offset
        return result


class FlockAlign(SteeringBehaviour):
    """Steering force for ALIGN group behaviour (flocking).

    Args:
        owner (SimpleVehicle2d): The vehicle computing this force.

    Notes:
        Unlike(?) traditional boids, we ALIGN with the average of neighbors'
        velocity vectors. Align with heading (normalize velocity) looked weird.
    """
    def __init__(self, owner):
        SteeringBehaviour.__init__(self, owner)
        self.owner = owner
        owner.flocking = True

    def force(self):
        result = Point2d(0,0)
        n = 0
        for other in self.owner.neighbor_list:
            if other is not self.owner:
                result += other.vel
                n += 1
        if n > 0:
            return (1.0/n)*result - self.owner.front
        else:
            return ZERO_VECTOR


class FlockCohesion(SteeringBehaviour):
    """Steering force for COHESION group behaviour.

    Args:
        owner (SimpleVehicle2d): The vehicle computing this force.
    """

    def __init__(self, owner):
        SteeringBehaviour.__init__(self, owner)
        self.owner = owner
        owner.flocking = True

    def force(self):
        center = Point2d(0,0)
        n = 0
        for other in self.owner.neighbor_list:
            if other is not self.owner:
                center += other.pos
                n += 1
        if n > 0:
            # ARRIVE at the average position of flock neighbors
            target_offset = ((1.0/n)*center - self.owner.pos)
            dist = target_offset.norm()
            if dist > 0:
                speed = dist / (ARRIVE_DECEL_TWEAK * FLOCKING_COHESION_HESITANCE)
                if speed > self.owner.maxspeed:
                    speed = self.owner.maxspeed
                targetvel = (speed/dist)*target_offset
                return targetvel - self.owner.vel
        # If no neighbors or already at flock center, no force is needed
        return ZERO_VECTOR


########################################################
### Navigator-type class to control vehicle steering ###
########################################################
class Navigator(object):
    """Helper class for managing steering behaviours.

    Args:
        vehicle (SimpleVehicle2d): The vehicle to be steered.
        use_budget (boolean): If True (default), use prioritized budgeted force.
    """

    # Dictionary of defined behaviours for later use
    _steering = {beh.__name__.upper(): beh for beh in SteeringBehaviour.__subclasses__()}

    def __init__(self, vehicle, use_budget=True):
        self.vehicle = vehicle
        self.steering_force = Point2d(0,0)
        self.active_behaviours = dict()
        self.paused_behaviours = dict()
        if use_budget:
            self.force_update = Navigator.compute_force_budgeted
            self.sorted_behaviours = None
        else:
            self.force_update = Navigator.compute_force_simple
        vehicle.navigator = self
        vehicle.flocking = False
        # TODO: Give vehicle convenient access to navigator interface,
        #       such as vehicle.set_steering = self.set_steering(...)

    def set_steering(self, behaviour, *args):
        """Add a new steering behaviour or change existing targets.

        Args:
            behaviour(string): Name of the behaviour to be added/modified.
            args: Additional arguments for that behaviour.
        """
        # First make sure that the behaviour is defined.
        try:
            steer_class = Navigator._steering[behaviour]
        except KeyError:
            logging.warning('set_steering: %s is not available.' % behaviour)
            return False
        newsteer = steer_class(self.vehicle, *args)
        # If this behaviour isn't already in use, mark for later sorting.
        if behaviour not in self.active_behaviours:
            # If we're using budgeted force, this ensures the extended list of
            # active_behaviours gets sorted by priority before the next update.
            # Since multiple new behaviours are often added simultaenously,
            # this ensures we only need a single sort.
            if self.force_update == Navigator.compute_force_budgeted:
                self.force_update = Navigator.sort_budget_priorities
        self.active_behaviours[behaviour] = newsteer

    def pause_steering(self, behaviour):
        """Temporarilily turns off a steering behaviour, to be resumed later.

        Args:
            behaviour(string): Name of the behaviour to be paused.
        """
        try:
            beh = self.active_behaviours.pop(behaviour)
        except KeyError:
            logging.info('Behaviour %s is not active; ignoring pause.' % behaviour)
            return False
        self.paused_behaviours[behaviour] = beh
        return True

    def resume_steering(self, behaviour):
        """Reactivates a previously paused behaviour with prior targets.

        Args:
            behaviour(string): Name of the behaviour to be resumed.
        """
        try:
            beh = self.paused_behaviours.pop(behaviour)
        except KeyError:
            logging.info('Behaviour %s is not paused; ignoring resume.' % behaviour)
            return False
        self.active_behaviours[behaviour] = beh
        if self.force_update == Navigator.compute_force_budgeted:
            self.force_update = Navigator.sort_budget_priorities
        return True

    def update(self, delta_t=1.0):
        """Update neighbors if needed; compute/apply steering force and move.

        Args:
            delta_t (float, optional): Time since last update; currently unused.

        Todo:
            It may be desirable to seperate flocking/nagivation/move updates
            into their own functions, since not all of these may occur at the
            same rate (possibly for performance reasons, for example).
        """
        if self.vehicle.flocking:
            self.update_neighbors(self.vehicle.flockmates)
        self.force_update(self)

    def compute_force_simple(self):
        """Updates the current steering force using all active behaviors.

        Note:
            Since the vehicle classes are expected to limit the maximum force
            applied, this function no longer does so.
        """
        self.steering_force.zero()
        # Iterate over active behaviours and accumulate force from each
        for behaviour in self.active_behaviours.values():
            self.steering_force += behaviour.force()

    def compute_force_budgeted(self):
        """Find prioritized steering force within the vehicle's budget.

        Todo:
            Vehicle's maxforce is used as the budget. Allow other values?
        """
        self.steering_force.zero()
        budget = self.vehicle.maxforce
        for behaviour in self.active_behaviours.values():
            newforce = behaviour.force()
            newnorm = newforce.norm()
            if budget > newnorm:
                # If there is enough force budget left, continue as usual
                self.steering_force += newforce
                budget -= newnorm
            else:
                # Scale newforce to remaining budget, apply, and exit
                self.steering_force += (budget/newnorm)*newforce
                return self.steering_force

        # If any budget is leftover, just return the total force
        return self.steering_force

    def sort_budget_priorities(self):
        """Sort our currently-active behaviours by priority; see below.

        This is intended to work with budgeted force. When a new behaviour is
        added, the Navigator's force_update method is set to this function, and
        the new list of behaviours gets sorted just before the actual update.
        One can also call this manually to immediately perform the sort. In any
        case, the force_update is then reset to compute_force_budgeted.
        """
        self.sorted_behaviours = OrderedDict(sorted(self.active_behaviours.items(),
                                                    key=PRIORITY_KEY))
        self.active_behaviours = self.sorted_behaviours
        self.force_update = Navigator.compute_force_budgeted
        self.force_update(self)

    def update_neighbors(self, vehlist=[]):
        """Populates a list of nearby vehicles, for use with flocking.

        Args:
            vehlist (List of BasePointMass2d): Objects to check; see notes.

        Notes:
            This function checks object based on their distance to owner and
            includes only those in front of the owner. Maximum distance is the
            owner's radius times FLOCKING_RADIUS_MULTIPLIER. We may consider
            more sophisticated sensing of neighbors in the future.

            Any pre-processing (spatial partitioning, sensory perception, or
            flocking with certain vehicles only) should be done before calling
            this function; with those results passed in as vehlist.

            Results are stored as owner.neighbor_list to be read later by any
            steering force() functions (mostly flocking) that require neighbor
            information. Run this function before each steering update.

        Todo:
            See Notes above for possible updates to this function.
        """
        owner = self.vehicle
        n_radius = owner.radius * FLOCKING_RADIUS_MULTIPLIER
        owner.neighbor_list = list()
        for other in vehlist:
            if other is not owner:
                min_range_sq = (n_radius + other.radius)**2
                offset = other.pos - owner.pos
                if offset.sqnorm() < min_range_sq:
                    # Only consider neighbors to the front
                    if offset*owner.front >= 0:
                        owner.neighbor_list.append(other)

if __name__ == "__main__":
    print("\nValues of imported steering constants are:")
    for cname in sorted(STEERING_DEFAULTS.keys()):
        try:
            print('%s : %.4f' % (cname, vars()[cname]))
        except KeyError:
            print('%s not defined!' % cname)
