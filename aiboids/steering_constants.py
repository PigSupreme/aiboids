# steering_constants.py
"""
Default values of constants for vehicles and steering behaviours.

Todo:
    Finish documenting default constants.
"""

from __future__ import print_function

########################
## Vehicle2d Defaults ##
########################

#: A BasePointMass2d has velocity-aligned heading. However, if the speed is
#: almost zero (squared speed is below this threshold), we skip alignment in
#: order to avoid jittery behaviour.
SPEED_EPSILON_SQ = 0.000000001

#: Vehicle mass for rectilinear motion.
POINTMASS2D_MASS = 1.0
#: Maximum vehicle speed per rectilinear motion update.
POINTMASS2D_MAXSPEED = 8.0
#: Maximum force/budget per rectilinear motion update.
POINTMASS2D_MAXFORCE = 6.0

#: Rotational Inertia for rigid-body physics
RIGIDBODY2D_INERTIA = 1.0
#: Maximum angular velocity per rigid-body update.
RIGIDBODY2D_MAXOMEGA = 90.0
#: Maximum torque per rigid-body update.
RIGIDBODY2D_MAXTORQUE = 75.0


#######################
## Steering defaults ##
#######################

#: ARRIVE global tweaking: larger values give more gradual deceleration.
ARRIVE_DECEL_TWEAK = 10.0
#: Default ARRIVE hesitance on a per-instance basis
ARRIVE_DEFAULT_HESITANCE = 2.0

#: Default forward distance to center of WANDER circle
WANDER_DISTANCE = 30.0
#: Default radius of WANDER circle
WANDER_RADIUS = 25.0
#: Default jitter of WANDER circle
WANDER_JITTER = 15.0

#: This controls the size of an object detection box for AVOID obstacles.
#: Length in front of vehicle is 100%-200% of this.
OBSTACLEAVOID_MIN_LENGTH = 120.0
#: Tweaking constant for braking force of AVOID obstacles.
OBSTACLEAVOID_BRAKE_WEIGHT = 0.01

#: TAKECOVER: For stalking, set this to cos^2(theta), where theta is the max
#: angle from target's front vector. The stalker will not hide unless within
#: this angle of view.
TAKECOVER_STALK_COS = 2**(.5)  # cos(45 degrees)
#:
TAKECOVER_STALK_DSQ = 100.0**2
#:
TAKECOVER_EVADE_MULT = 1.5
#:
TAKECOVER_ARRIVE_HESITANCE = 1.0

#: WALLAVOID: Proportional length of side whiskers relative to front whisker.
WALLAVOID_SIDE_SCALE = 0.8

#: This is cos(10 degrees)
PURSUE_POUNCE_COS = 0.966
#:
PURSUE_POUNCE_DISTANCE = 100.0

#: FOLLOW the leader uses ARRIVE with this hesitance, for smooth formations.
FOLLOW_ARRIVE_HESITANCE = 1.5

#: EVADE ignores the predator beyond this distance
EVADE_PANIC_DIST = 160.0

#: GUARD uses this hesitance to arrive at the guard point.
GUARD_HESITANCE = 1.0

#: SteeringPath will treat consecutive waypoints that are closer than this
#: as duplicates, and remove them from the path.
PATH_EPSILON_SQ = 10.0**2

#: Used by PATHFOLLOW/RESUME to determine when we're close enough to a waypoint.
WAYPOINT_RADIUS = 10.0

#: Exponential decay constant for PATHRESUME.
PATHRESUME_DECAY = 0.075

#: For simplicity, we multiply the vehicle's bounding radius by this constant
#: to determine the local neighborhood radius for group behaviours.
FLOCKING_RADIUS_MULTIPLIER = 5.0

#: Scaling factor for SEPERATE group behaviour.
#: Larger values give greater seperation force.
FLOCKING_SEPARATE_SCALE = 2.1

#: Cohesion essentially ARRIVEs with this hesitance, for smooth flocking.
FLOCKING_COHESION_HESITANCE = 3.5

#########################################
## Encapsulated imports below
#########################################

#: Point-Mass physics defaults.
BASEPOINTMASS2D_DEFAULTS = {
    'MASS': POINTMASS2D_MASS,
    'MAXSPEED': POINTMASS2D_MAXSPEED,
    'MAXFORCE': POINTMASS2D_MAXFORCE,
    }

#: Additional Rigid Body physics defaults.
SIMPLERIGIDBODY2D_DEFAULTS = {
    'INERTIA':  RIGIDBODY2D_INERTIA,
    'MAXOMEGA': RIGIDBODY2D_MAXOMEGA,
    'MAXTORQUE': RIGIDBODY2D_MAXTORQUE
    }
SIMPLERIGIDBODY2D_DEFAULTS.update(**BASEPOINTMASS2D_DEFAULTS)

#: Defaults for Steering Behaviours.
STEERING_DEFAULTS = {
    'ARRIVE_DECEL_TWEAK': ARRIVE_DECEL_TWEAK,
    'ARRIVE_DEFAULT_HESITANCE': ARRIVE_DEFAULT_HESITANCE,
    'WANDER_DISTANCE': WANDER_DISTANCE,
    'WANDER_RADIUS': WANDER_RADIUS,
    'WANDER_JITTER': WANDER_JITTER,
    'WALLAVOID_SIDE_SCALE': WALLAVOID_SIDE_SCALE,
    'OBSTACLEAVOID_BRAKE_WEIGHT': OBSTACLEAVOID_BRAKE_WEIGHT,
    'OBSTACLEAVOID_MIN_LENGTH': OBSTACLEAVOID_MIN_LENGTH,
    'TAKECOVER_STALK_COS': TAKECOVER_STALK_COS,
    'TAKECOVER_STALK_DSQ': TAKECOVER_STALK_DSQ,
    'TAKECOVER_EVADE_MULT': TAKECOVER_EVADE_MULT,
    'TAKECOVER_ARRIVE_HESITANCE': TAKECOVER_ARRIVE_HESITANCE,
    'PURSUE_POUNCE_COS': PURSUE_POUNCE_COS,
    'PURSUE_POUNCE_DISTANCE': PURSUE_POUNCE_DISTANCE,
    'FOLLOW_ARRIVE_HESITANCE': FOLLOW_ARRIVE_HESITANCE,
    'EVADE_PANIC_DIST': EVADE_PANIC_DIST,
    'GUARD_HESITANCE': GUARD_HESITANCE,
    'WAYPOINT_RADIUS': WAYPOINT_RADIUS,
    'PATH_EPSILON_SQ': PATH_EPSILON_SQ,
    'PATHRESUME_DECAY': PATHRESUME_DECAY,
    'FLOCKING_RADIUS_MULTIPLIER': FLOCKING_RADIUS_MULTIPLIER,
    'FLOCKING_COHESION_HESITANCE': FLOCKING_COHESION_HESITANCE,
    'FLOCKING_SEPARATE_SCALE': FLOCKING_SEPARATE_SCALE
    }

#: Order in which behaviours are considered when using budgeted force:
PRIORITY_DEFAULTS = [
    'BRAKE',
    'WALLAVOID',
    'OBSTACLEAVOID',
    'FLOCKSEPARATE',
    'FLEE',
    'EVADE',
    'SEEK',
    'ARRIVE',
    'TAKECOVER',
    'PURSUE',
    'GUARD',
    'FOLLOW',
    'WAYPATHRESUME',
    'WAYPATHVISIT',
    'FLOCKCOHESION',
    'FLOCKALIGN',
    'FLOWFOLLOW',
    'WANDER'
    ]

if __name__ == "__main__":
    print("Steering constants. Import this elsewhere. Default values below.")
    print("\n  SPEED_EPSILON_SQ = %s" % SPEED_EPSILON_SQ)
    for dlist in (BASEPOINTMASS2D_DEFAULTS, SIMPLERIGIDBODY2D_DEFAULTS, STEERING_DEFAULTS):
        print("")
        for k in sorted(dlist):
            print("  %s = %s" % (k, dlist[k]))
