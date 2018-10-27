# aiboids/steering.py
"""Module containing AiBoid steering behavior functions (and Navigator).

Many behaviours use constant values, imported from steering_constants.py as the
STEERING_DEFAULTS dictionary. These are chosen based on the physics defaults
(also in steering_constants) to give reasonable results.

Todo:
    Best way to override STEERING_DEFAULTS? Many of these constants are used as
    [immutable] default values in the SteeringBehvaiour.__init__() functions,
    so simply redefining their values won't work. One possible solution is to
    provide a mechanism for modifying foo.__init__.__defaults__, perhaps in the
    abstract base class.

Todo:
    Document keyword arguments; these are usually for overriding default values
    from steering.constants.py.

Todo:
    See Navigator.update_neighbors() docstring notes for possible updates.
"""

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

# Dictionary of imported default constant values
from aiboids.steering_constants import STEERING_DEFAULTS

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
        # Computes the owner's steering force for this behaviour.
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
    constants = {'DEFAULT_HESITANCE': STEERING_DEFAULTS['ARRIVE_DEFAULT_HESITANCE'],
                 'DECEL_TWEAK': STEERING_DEFAULTS['ARRIVE_DECEL_TWEAK']
                }

    def __init__(self, owner, target, hesitance=constants['DEFAULT_HESITANCE']):
        SteeringBehaviour.__init__(self, owner)
        self.target = target
        self.hesitance = hesitance * Arrive.constants['DECEL_TWEAK']

    def force(self):
        owner = self.owner
        target_offset = (self.target - owner.pos)
        dist = target_offset.norm()
        if dist > 0:
            speed = dist / self.hesitance
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

    Todo:
        Give each instance its own random number generator for jitter; this
        will make testing more reliable.
    """
    constants = {'DEFAULT_DISTANCE' : STEERING_DEFAULTS['WANDER_DISTANCE'],
                 'DEFAULT_RADIUS' : STEERING_DEFAULTS['WANDER_RADIUS'],
                 'DEFAULT_JITTER': STEERING_DEFAULTS['WANDER_JITTER']}

    def __init__(self, owner,
                 distance=constants['DEFAULT_DISTANCE'],
                 radius=constants['DEFAULT_RADIUS'],
                 jitter=constants['DEFAULT_JITTER']):
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
    constants = {'MIN_LENGTH': STEERING_DEFAULTS['OBSTACLEAVOID_MIN_LENGTH'],
                 'BRAKE_WEIGHT': STEERING_DEFAULTS['OBSTACLEAVOID_BRAKE_WEIGHT']}

    def __init__(self, owner, obstacle_list, *,
                 min_length=constants['MIN_LENGTH'],
                 brake_weight=constants['BRAKE_WEIGHT']):
        SteeringBehaviour.__init__(self, owner)
        self.obstacles = tuple(obstacle_list)
        self.min_length = min_length
        self.brake_weight = brake_weight

    def force(self):
        owner = self.owner
        # Obstacles closer than this distance will be avoided
        front_d = (1 + owner.vel.norm()/owner.maxspeed)* self.min_length
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
            brake_scale = (lr - lx)*self.brake_weight
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

    Keyword Args:
        hesitance (positive float): Hesitance for arriving at the hiding spot.
        evade_mult (float): Used to compute EVADE panic distance; see below.
        stalk (boolean): If True, only hide when we're detectable; see below.
        stalk_dsq(positive float): See stalk description below.
        stalk_cos(float): See stalk description below.

    Owner attempts to find a spot that puts an obstacle between itself and the
    target vehicle, then ARRIVEs at that position. If a spot cannot be found
    nearby (within max_range of the owner), EVADE the target instead, using a
    panic distance of evade_mult*max_range.

    If stalk is set to True, the owner takes cover only when close enough to the
    target and within a maximum angle of the target's front vector. These values
    are controlled by STALK_DSQ and STALK_COS, respectively.
    """
    constants = {'EVADE_MULT': STEERING_DEFAULTS['TAKECOVER_EVADE_MULT'],
                 'HESITANCE': STEERING_DEFAULTS['TAKECOVER_ARRIVE_HESITANCE'],
                 'OBSTACLE_PROXIMITY': STEERING_DEFAULTS['TAKECOVER_OBSTACLE_PROXIMITY'],
                 'STALK_DSQ': STEERING_DEFAULTS['TAKECOVER_STALK_DSQ'],
                 'STALK_COS': STEERING_DEFAULTS['TAKECOVER_STALK_COS'],
                 'DECEL_TWEAK': STEERING_DEFAULTS['ARRIVE_DECEL_TWEAK']}

    def __init__(self, owner, target, obstacle_list, max_range, *,
                 hesitance=constants['HESITANCE'],
                 evade_mult=constants['EVADE_MULT'],
                 stalk=False,
                 stalk_dsq=constants['STALK_DSQ'],
                 stalk_cos=constants['STALK_COS']):
        SteeringBehaviour.__init__(self, owner)
        self.target = target
        self.obstacles = tuple(obstacle_list)
        self.range_sq = max_range**2
        self.proximity = TakeCover.constants['OBSTACLE_PROXIMITY']*owner.radius
        self.stalk = stalk
        if stalk:
            self.stalk_dsq = stalk_dsq
            self.stalk_cos = stalk_cos
        # A helper instance of EVADE to avoid duplicating code here.
        self.evade_helper = Evade(owner, target, evade_mult*max_range)
        # ARRIVE needs a fixed position as its target; t's not worth using a
        # seperate instance here. But we still need the effective hesistance.
        self.hesitance = hesitance * TakeCover.constants['DECEL_TWEAK']

    def force(self):
        best_pos = None
        # If stalking and not within the target's detectable area, our desired
        # position (best_pos) is just behind the target.
        if self.stalk:
            hide_dir = (self.owner.pos - self.target.pos)
            if hide_dir.sqnorm() > self.stalk_dsq or (hide_dir.unit()*self.target.front) < self.stalk_cos:
                best_pos = self.target.pos - self.proximity*self.target.front

        # Otherwise, try to find the nearest hiding spot bethind an obstacle.
        if not best_pos:
            best_dsq = self.range_sq
            for obs in self.obstacles:
                # Find the hiding spot if we use this obstacle; directly
                # opposite the vehicle we take cover from.
                hide_dir = (obs.pos - self.target.pos).unit()
                hide_pos = obs.pos + (obs.radius + self.proximity)*hide_dir
                hide_dsq = (hide_pos - self.owner.pos).sqnorm()
                if hide_dsq < best_dsq:
                    best_pos = hide_pos
                    best_dsq = hide_dsq

        # If there's no hiding/stalking spot, EVADE the target...
        if not best_pos:
            return self.evade_helper.force()

        # ...otherwise, ARRIVE at the given spot.
        target_offset = (best_pos - self.owner.pos)
        dist = target_offset.norm()
        if dist > 0:
            speed = min(dist / self.hesitance, self.owner.maxspeed)
            targetvel = (speed/dist)*target_offset
            return targetvel - self.owner.vel

        return ZERO_VECTOR


class WallAvoid(SteeringBehaviour):
    """WALLAVOID behaviour using simulated whiskers.

    Args:
        owner (SimpleVehicle2d): The vehicle computing this force.
        whikser_list (list of Point2d): Whisker vectors in owner local space.
        wall_list (list, BaseWall2d): List of walls to test against.

    For each whisker, we find the wall having its point of intersection
    closest to the base of the whisker. If such a wall is detected, it
    contributes a force in the direction of the wall normal, proportional
    to the penetration depth of the whisker.
    """
    def __init__(self, owner, whisker_list, wall_list=None):
        SteeringBehaviour.__init__(self, owner)
        #self.whisker_coords = ((1,0), (SQRT_HALF,SQRT_HALF), (SQRT_HALF,-SQRT_HALF))
        #side_length = front_length * side_scale
        self.whisker_num = len(whisker_list)
        self.whisker_sizes = [w.norm() for w in whisker_list]
        self.whisker_dirs = [w.unit() for w in whisker_list]
        self.walls = wall_list

    def force(self):
        owner = self.owner
        whisker_tip = self.whisker_num*[0]
        closest_wall = self.whisker_num*[None]

        # Convert unit vectors for each whisker to global coordinates
        for i in range(self.whisker_num):
            (u,v) = self.whisker_dirs[i]
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
    """PURSUE a moving object; the opposite of EVADE.

    Args:
        owner (SimpleVehicle2d): The vehicle computing this force.
        prey (BasePointMass2d): The object we're pursuing.
        pcos (float, optional): Cosine of the pounce angle, see below.
        pdist (float, optional): Pounce distance, see below.

    If the prey is heading our way (we are within a certain angle of
    the prey's heading) and within a certain distance, simply "pounce"
    on the prey by SEEKing to its current position. Otherwise, predict
    the future position of they prey, based on current velocities,
    and SEEK to that location.
    """
    constants = {'POUNCE_COS': STEERING_DEFAULTS['PURSUE_POUNCE_COS'],
                 'POUNCE_DISTANCE': STEERING_DEFAULTS['PURSUE_POUNCE_DISTANCE']}

    def __init__(self, owner, prey, pcos=constants['POUNCE_COS'], pdist=constants['POUNCE_DISTANCE']):
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
    constants = {'ARRIVE_HESITANCE': STEERING_DEFAULTS['FOLLOW_ARRIVE_HESITANCE'],
                 'DECEL_TWEAK': STEERING_DEFAULTS['ARRIVE_DECEL_TWEAK']}

    def __init__(self, owner, leader, offset, hesitance=constants['ARRIVE_HESITANCE']):
        SteeringBehaviour.__init__(self, owner)
        self.leader = leader
        self.offset = offset
        self.hesitance = hesitance * Follow.constants['DECEL_TWEAK']

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
            speed = dist / self.hesitance
            if speed > owner.maxspeed:
                speed = owner.maxspeed
            targetvel = (speed/dist)*offset
            return targetvel - owner.vel
        else:
            return ZERO_VECTOR


class Evade(SteeringBehaviour):
    """EVADE a moving object; the opposite of PURSUE.

        Args:
            owner (SimpleVehicle2d): The vehicle computing this force.
            predator (BasePointMass2d): The object we're evading.
            panic_dist (float): Ignore the predator beyond this distance.
    """
    constants = {'PANIC_DIST': STEERING_DEFAULTS['EVADE_PANIC_DIST']}
    def __init__(self, owner, predator, panic_dist=constants['PANIC_DIST']):
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
    constants = {'HESITANCE': STEERING_DEFAULTS['GUARD_HESITANCE']*STEERING_DEFAULTS['ARRIVE_DECEL_TWEAK']}

    def __init__(self, owner, guard_this, guard_from, aggro=0.5):
        SteeringBehaviour.__init__(self, owner)
        self.guard_this = guard_this
        self.guard_from = guard_from
        self.aggro = aggro
        self.hesitance = Guard.constants['HESITANCE']

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
            speed = dist / self.hesitance
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

    Todo:
        Once tested, add a BRAKE_DECAY constant to steering.defaults.
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
        waypoints (list of Point2d): At least two waypoints; see below.
        is_cyclic (boolean): If True, path will automatically cycle, returning
            to *waypoints[1]* after the last waypoint.

    When used for vehicle steering, an instance of this class should be owned
    by a vehicle Navigator instance, but all path-management code is controlled
    from here.

    *waypoints[0]* is intended as the starting point of the owner vehicle, and
    is ignored when the path is reset or cycled. To include this point in a
    cyclic path (for example, a patrol route that returns to to start), add the
    starting point as the last element of *waypoints*.

    The *waypoints* list is pre-processed so that any point that is close
    to its predecessor (measured by _EPSILON_SQ) is ignored. If all points
    are close, we raise a ValueError.

    Raises:
        ValueError: If *waypoints* has fewer than two entries, or all points
            are close together (as described above).
    """
    _EPSILON_SQ = STEERING_DEFAULTS['PATH_EPSILON_SQ']
    _RADIUS = STEERING_DEFAULTS['WAYPOINT_RADIUS']

    def __init__(self, waypoints, is_cyclic=False):
        if len(waypoints) < 2:
            raise ValueError('At least two waypoints needed in a path.')
        self.oldway = waypoints[0]
        self.waypoints = []
        self.wayradius_sq = WaypointPath._RADIUS**2
        prev_wp = self.oldway

        # Include only consecutive waypoints that are far enough apart
        for wp in waypoints[1:]:
            if (prev_wp - wp).sqnorm() >= WaypointPath._EPSILON_SQ:
                self.waypoints.append(wp)
                prev_wp = wp
        # In the excpetional case that all waypoints are close together...
        if self.waypoints == []:
            raise ValueError('Cannot initialize path; waypoints are too close.')

        # Compute initial segment, see Notes on returning to first waypoint
        self.newway = self.waypoints[0]
        self.wpindex = 0

        # Length of this edge and unit vector (oldway to newway)
        offset = self.newway - self.oldway
        self.edgelength = offset.norm()
        self.edgevector = (1.0/self.edgelength)*offset

        self.is_cyclic = is_cyclic

    def restart_from(self, new_start_pos, start_index=0):
        """Explicitly set the next waypoint and follow the path from there.

        Args:
            start_pos (Point2d): Starting location for path following.
            start_index (positive int): Index of the next waypoint.

        As with __init__(), start_pos is intended as the current location of
        some vehicle, and is not explicitly added to the waypoint list.

        Note:
            If start_pos is close (measured by _EPSILON_SQ) to the given
            waypoint, we ignore that waypoint and start from the next one.
        """
        self.newway = self.waypoints[start_index]
        self.wpindex = start_index
        # If we're close to the first waypoint, ignore it and use start_pos
        if (new_start_pos - self.newway).sqnorm() < WaypointPath._EPSILON_SQ:
            self.advance()
            self.oldway = new_start_pos

    def resume_at_nearest_from(self, from_pos, ignore_visited=True):
        """Find the closest waypoint and follow the path from there; see notes.

        Args:
            from_pos (Point2d): The new starting point for the path.
            ignore_visited (boolean): If True (default), only consider those
                waypoints not yet visited when finding the closest. Has no
                effect on cyclic paths.
        """
        if ignore_visited:
            start = self.wpindex
        else:
            start = 1
        self.wpindex = min(range(start, len(self.waypoints)), key=lambda i: (self.waypoints[i]-from_pos).sqnorm())
        self.newway = self.waypoints[self.wpindex]
        # If we're within the arrive radius of this waypoint, use the next one
        if (self.newway - from_pos).sqnorm() < self.wayradius_sq:
            self.advance()
        else:
            self.oldway = self.waypoints[self.wpindex - 1]


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
        return len(self.waypoints) - self.wpindex


class WaypathVisit(SteeringBehaviour):
    """Visit a series of waypoints in order.

    Args:
        owner (SimpleVehicle2d): The vehicle computing this force.
        waypath (WaypointPath): The path to be followed.
        wayradius (float): Waypoint radius; see below.

    A waypoint is visited once the distance to owner is less than *wayradius*.
    In this version; we steer directly at the next waypoint, even if we are
    knocked off-course. See WayPathResume for an alternative.

    For the final waypoint, we ARRIVE at it and stay there as long as this
    behaviour remains active. For previous waypoints, we use SEEK.
    """
    def __init__(self, owner, waypath, wayradius=WaypointPath._RADIUS):
        SteeringBehaviour.__init__(self, owner)
        self.waypath = waypath
        self.wprad_sq = wayradius**2
        self.nextwaypt = waypath.newway
        if self.nextwaypt is None:
            raise ValueError('No more waypoints to visit.')
        if waypath.num_left() > 1:
            self.wayforce = Seek(owner, self.nextwaypt)
        else:
            self.wayforce = Arrive(owner, self.nextwaypt)


    def force(self):
        # If there's a current destination, check if we're within range of it
        if self.nextwaypt and (self.owner.pos - self.nextwaypt).sqnorm() <= self.wprad_sq:
            self.waypath.advance()
            self.nextwaypt = self.waypath.newway
            # If this was the last waypoint in the path, we should already be
            # using ARRIVE, and don't need to change anything. Otherwise we
            # update the steering behaviour to the new waypoint.
            if self.nextwaypt is not None:
                del self.wayforce
                if self.waypath.num_left() == 1:
                    self.wayforce = Arrive(self.owner, self.nextwaypt)
                else:
                    self.wayforce = Seek(self.owner, self.nextwaypt)
        return self.wayforce.force()


class WaypathResume(SteeringBehaviour):
    """Visit waypoints in order, trying to stay close to the path between them.

    Args:
        owner (SimpleVehicle2d): The vehicle computing this force.
        waypath (WaypointPath): The path to be followed.
        expk (positive float): Exponential decay constant; see below.
        wayradius (float): Waypoint radius; see below.

    If the vehicle is knocked off-course, this will give a balance between
    returning directly to the current path edge and progressing towards the next
    waypoint; this is controlled by *expk*. Larger values give a more immediate
    return to the path.

    Notes:
        If the vehicle has already overshot the next waypoint, we head directly
        to that waypoint, ignoring the path. Otherwise, follow an exponential
        decay curve asymptotic to the path; although this curve doesn't truly
        pass through the waypoint, it makes computations very quick, especially
        since we store *invk*. Since, a waypoint is visited once the we're
        withint *wayradius*, we don't need to hit the waypoint exactly.
    """
    constants = {'EXP_DECAY': STEERING_DEFAULTS['PATHRESUME_EXP_DECAY'],
                 'DECEL_TWEAK': STEERING_DEFAULTS['ARRIVE_DECEL_TWEAK']}

    def __init__(self, owner, waypath, expk=constants['EXP_DECAY'],
                 wayradius=WaypointPath._RADIUS):
        SteeringBehaviour.__init__(self, owner)
        self.waypath = waypath
        self.invk = 1.0/expk
        self.wprad_sq = wayradius**2
        self.hesitance = WaypathResume.constants['DECEL_TWEAK']

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
                    speed = min(dist / self.hesitance, owner.maxspeed)
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

    Warning:
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
class FlockSeparate(SteeringBehaviour):
    """SEPARATE from our neighbors to avoid collisions (flocking).

    Args:
        owner (SimpleVehicle2d): The vehicle computing this force.

    Notes:
        For each neighbor, include a force away from that neighbor of magnitude
        proportional to the neighbor radius and inversely proprotional to
        distance. This gave nicer results and allows us to cleverly avoid
        computing a sqrt.
    """
    constants = {'FORCE_SCALE': STEERING_DEFAULTS['FLOCKING_SEPARATE_SCALE']}
    NEEDS_NEIGHBORS = True

    def __init__(self, owner, scale=constants['FORCE_SCALE']):
        SteeringBehaviour.__init__(self, owner)
        self.owner = owner
#        owner.flocking = True
        self.scale = scale

    def force(self):
        result = Point2d(0,0)
        for other in self.owner.neighbor_list:
            if other is not self.owner:
                offset = self.owner.pos - other.pos
                result += (self.scale*other.radius/offset.sqnorm())*offset
        return result


class FlockAlign(SteeringBehaviour):
    """ALIGN with our neighbors by matching their average velocity (flocking).

    Args:
        owner (SimpleVehicle2d): The vehicle computing this force.
    """
    NEEDS_NEIGHBORS = True

    def __init__(self, owner):
        SteeringBehaviour.__init__(self, owner)
        self.owner = owner
#        owner.flocking = True

    def force(self):
        result = Point2d(0,0)
        n = 0
        for other in self.owner.neighbor_list:
            if other is not self.owner:
                result += other.vel
                n += 1
        if n > 0:
            return (1.0/n)*result - self.owner.front
        return ZERO_VECTOR


class FlockCohesion(SteeringBehaviour):
    """COHESION with our neighbors towards their average position (flocking).

    Args:
        owner (SimpleVehicle2d): The vehicle computing this force.
    """
    constants = {'HESITANCE': STEERING_DEFAULTS['FLOCKING_COHESION_HESITANCE'],
                 'DECEL_TWEAK': STEERING_DEFAULTS['ARRIVE_DECEL_TWEAK']}
    NEEDS_NEIGHBORS = True

    def __init__(self, owner, hesitance=constants['HESITANCE']):
        SteeringBehaviour.__init__(self, owner)
        self.owner = owner
#        owner.flocking = True
        self.hesitance = hesitance*FlockCohesion.constants['DECEL_TWEAK']

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
                speed = dist / self.hesitance
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
        use_budget (boolean): If True (default), use prioritized budgeted force,
            with budget set to vehicle.maxforce
    """
    _FLOCK_SCALE = STEERING_DEFAULTS['FLOCKING_RADIUS_MULTIPLIER']
    # Dictionary of defined behaviours for later use
    _steering = {beh.__name__.upper(): beh for beh in SteeringBehaviour.__subclasses__()}

    def __init__(self, vehicle, use_budget=True):
        self.vehicle = vehicle
        self.steering_force = Point2d(0,0)
        self.active_behaviours = dict()
        self.paused_behaviours = dict()
        # Used to determine if we need neighbor updates
        self.active_flocking = 0
        if use_budget:
            self.force_update = Navigator.compute_force_budgeted
            self.sorted_behaviours = None
            self.force_budget = vehicle.maxforce
        else:
            self.force_update = Navigator.compute_force_simple
        vehicle.navigator = self

    def set_steering(self, behaviour, *args, **kwargs):
        """Add a new steering behaviour or change existing targets.

        Args:
            behaviour(string): Name of the behaviour to be added/modified.

        Additional positional and keyword arguments are passed as-is to the
        __init__() method of the given behaviour.
        """
        # First make sure that the behaviour is defined.
        try:
            steer_class = Navigator._steering[behaviour]
        except KeyError:
            logging.warning('set_steering: %s is not available.' % behaviour)
            return False
        newsteer = steer_class(self.vehicle, *args, **kwargs)
        # If this behaviour isn't already in use, mark for later sorting.
        if behaviour not in self.active_behaviours:
            # If we're using budgeted force, this ensures the extended list of
            # active_behaviours gets sorted by priority before the next update.
            # Since multiple new behaviours are often added simultaenously,
            # this ensures we only need a single sort.
            if self.force_update == Navigator.compute_force_budgeted:
                self.force_update = Navigator.sort_budget_priorities
            # Update count of active flocking behaviours
            if hasattr(steer_class,'NEEDS_NEIGHBORS'):
                self.active_flocking += 1
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
        if hasattr(beh.__class__, 'NEEDS_NEIGHBORS'):
            self.active_flocking -= 1
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
        # See comments in set_steering for an explanation of this
        if self.force_update == Navigator.compute_force_budgeted:
            self.force_update = Navigator.sort_budget_priorities
        if hasattr(beh.__class__, 'NEEDS_NEIGHBORS'):
            self.active_flocking += 1
        return True

    def update(self, delta_t=1.0):
        """Update neighbors (if needed) and set our new steering force.

        Args:
            delta_t (float, optional): Time since last update; currently unused.
        """
        # If at least one flocking behaviour is active, update neighbors
        if self.active_flocking:
            self.update_neighbors(self.vehicle.flockmates)
        self.force_update(self)

    def compute_force_simple(self):
        """Compute/update steering force using all active behaviors.

        Note:
            Since the BasePoint classes are expected to limit the maximum force
            applied, this function no longer does so.
        """
        self.steering_force.zero()
        # Iterate over active behaviours and accumulate force from each
        for behaviour in self.active_behaviours.values():
            self.steering_force += behaviour.force()
        return self.steering_force

    def compute_force_budgeted(self):
        """Compute/update prioritized steering force within a given budget."""
        self.steering_force.zero()
        budget = self.force_budget
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

    def update_neighbors(self, vehlist, radius_scale=_FLOCK_SCALE):
        """Populates a list of nearby vehicles, for use with flocking.

        Args:
            vehlist (List of BasePointMass2d): Objects to check; see notes.
            radius_scale (positive float): Check for neighbors within the
                owner's radius times this value.

        Notes:
            This function checks object based on their distance to owner and
            includes only those in front of the owner. Maximum distance is the
            owner's radius times radius_scale. We may use more sophisticated
            sensing of neighbors in the future.

            Any pre-processing (spatial partitioning, sensory perception, or
            flocking with certain vehicles only) should be done before calling
            this function; with those results passed in as vehlist.

            Results are stored as owner.neighbor_list to be read later by any
            steering force() functions (mostly flocking) that require neighbor
            information. Run this function before each steering update.
        """
        owner = self.vehicle
        n_radius = owner.radius * radius_scale
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
    for cname, cval in sorted(STEERING_DEFAULTS.items()):
        print('%s : %.4f' % (cname, cval))
