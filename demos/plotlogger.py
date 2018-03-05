#!/usr/bin/env python
"""Plotting demo with Corridor/lane traversal; no pygame."""

# for python3 compat
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import sys
INF = float('inf')

# Note: Adjust this depending on where this file ends up.
sys.path.append('..')
from aiboids.point2d import Point2d
from aiboids.vehicle2d import SimpleVehicle2d, BaseWall2d


if __name__ == "__main__":
    # Display set-up
    SCREEN_SIZE = (800,640)
    UPDATE_SPEED = 0.5

    # Waypoint/corridor information
    OFFSETX, OFFSETY = 80, 15
    corlen = SCREEN_SIZE[0] - 2*OFFSETX
    midx = SCREEN_SIZE[0]//2
    midy = SCREEN_SIZE[1]//2
    WALL_THICK = 5

    # Initial placement/velocity
    LEAD_OFFSET = 100
    INIT_SPEED = 12
    WHISKER_FRONT = 25.0
    base_pos = Point2d(OFFSETX, midy)
    base_dir = Point2d(1.0,0.5).unit()
    init_pos = base_pos - LEAD_OFFSET*base_dir
    init_vel = INIT_SPEED*base_dir

    # Vehicle info
    green = SimpleVehicle2d(init_pos, 20, init_vel)
    green.maxspeed = INIT_SPEED

    # Static Walls (used for corridor/lane)
    wall_list = [BaseWall2d((midx, midy-OFFSETY), corlen, WALL_THICK, Point2d(0,1)),
                 BaseWall2d((midx, midy+OFFSETY), corlen, WALL_THICK, Point2d(0,-1))
                ]

    # Green Steering
    nav = green.navigator
    nav.set_steering('ARRIVE', Point2d(SCREEN_SIZE[0], midy), 1.0)
    nav.pause_steering('ARRIVE')
    nav.set_steering('WALLAVOID', WHISKER_FRONT, wall_list)
    nav.pause_steering('WALLAVOID')
    in_lane = False

    # Logging for end plot
    glog = []

    ### Main loop ###
    while green.pos.x < SCREEN_SIZE[0]-OFFSETX:
        # Update Vehicles (implicitly calls their Navigators)
        glog.append(green.pos.ntuple)
        green.move(UPDATE_SPEED)

        # Once we're in the corridor, activate ARRIVE
        if not in_lane and green.pos.x > OFFSETX:
            nav.resume_steering('ARRIVE')
            nav.resume_steering('WALLAVOID')
            in_lane = True

    from matplotlib.pyplot import plot
    plot([midx-corlen/2, midx+corlen/2], 2*[midy-(OFFSETY-WALL_THICK)],'k--')
    plot([midx-corlen/2, midx+corlen/2], 2*[midy+(OFFSETY-WALL_THICK)],'k--')
    plot([p[0] for p in glog], [p[1] for p in glog],'+')

