#!/usr/bin/env python
"""
This is the main exectuable for the westworld3 demo.
"""

# for python3 compat
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import print_function

import logging
logging.basicConfig(format='%(message)s', level=logging.INFO)

import sched
import sys
sys.path.append('..')
from importlib import import_module
from aiboids.base_entity import BaseEntity, DummyClock, PostOffice
import aiboids.statemachine # TODO: Needed only to display state logic
from gamedata import Characters, GameOver

##############################################################################

if __name__ == "__main__":
    # Initialize Manager-type objects:
    MASTER_CLOCK = DummyClock()
    MAXTIME = 200
    SCHEDULER = sched.scheduler(MASTER_CLOCK.time, MASTER_CLOCK.tick)
    POST_OFFICE = PostOffice(SCHEDULER)

    # Create and register entities
    ENTITY_LIST = []
    for (ent_name, sourcefile) in Characters.items():
        source_mod = import_module(sourcefile)
        cls = getattr(source_mod, source_mod.ENTITY_CLASS)
        entity = cls(ent_name)
        ENTITY_LIST.append(entity)

    # TODO: Move this to an appropriate logger
    aiboids.statemachine.State.print_state_events()

    # Start FSM logic: Must be done AFTER all entities are registered.
    for entity in ENTITY_LIST:
        entity.statemachine.start()


    def auto_tick(clock):
        """Used to schedule/propagate automatic clock and entity updates."""
        print('=== Clock time is now %d ===' % clock.time())
        if 1 + clock.time() <= MAXTIME:
            SCHEDULER.enter(1, 0, auto_tick, (clock,))
            SCHEDULER.enter(1, 0.5, BaseEntity.update_all)
        else:
            raise GameOver

    # Start the clock and let the scheduler do its thing
    auto_tick(MASTER_CLOCK)
    try:
        SCHEDULER.run()
    except GameOver:
        pass

    # Print some final game statistics
    print("\n === Final Results ===")
    print("Elapsed time: %d clock ticks." % MASTER_CLOCK.time())
    for entity in ENTITY_LIST:
        try:
            print('%s: %s' % (entity.name, entity.final_score()))
        except:
            pass
