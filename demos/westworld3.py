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

import sys
sys.path.append('..')
from importlib import import_module
from aiboids.base_entity import BaseEntity, DummyClock, PostOffice
from gamedata import Characters, GameOver

##############################################################################

if __name__ == "__main__":
    # Initialize Manager-type objects:
    MASTER_CLOCK = DummyClock()
    POST_OFFICE = PostOffice(MASTER_CLOCK)

    # Create and register entities
    ENTITY_LIST = []
    for (ent_name, sourcefile) in Characters.items():
        source_mod = import_module(sourcefile)
        cls = getattr(source_mod, source_mod.ENTITY_CLASS)
        entity = cls(ent_name)
        ENTITY_LIST.append(entity)

    # Start FSM logic: Must be done AFTER all entities are registered.
    for entity in ENTITY_LIST:
        entity.statemachine.start()

    # Main Loop
    while 1:
        MASTER_CLOCK.tick()
        print('=== Clock time is now %d ===' % MASTER_CLOCK.time())
        try:
            BaseEntity.update_all()
            POST_OFFICE.dispatch_queued()
        except GameOver:
            break
    print("\n === Final Results ===")
    print("Elapsed time: %d clock ticks." % MASTER_CLOCK.time())
    for entity in ENTITY_LIST:
        try:
            print('%s: %s' % (entity.name, entity.final_score()))
        except:
            pass
