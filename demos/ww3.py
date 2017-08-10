#!/usr/bin/env python
"""
This is the main exectuable for the westworld2 demo.
"""

# for python3 compat
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import print_function

#from fsm_ex.gamedata import Characters, Locations, MsgTypes
#from fsm_ex.gamedata import GameOver

import logging
logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)

import sys
sys.path.append('..')
from aiboids.base_entity import BaseEntity, DummyClock, PostOffice

from gamedata import GameOver

# TODO: Automate this so that we can just import something from gamedata.py?
from ent_miner import Miner
from ent_wife import Wife
#from fsm_ex.ent_goat import Goat


##############################################################################

if __name__ == "__main__":

    # Initialize Manager-type objects:
    MASTER_CLOCK = DummyClock()
    POST_OFFICE = PostOffice(MASTER_CLOCK)

    # Create and register entities
    BOB = Miner('MINER_BOB')
    ELSA = Wife('WIFE_ELSA')
    #BILLY = Goat('BILLY_GOAT') 

    # Start FSM logic: Must be done AFTER all entities are registered.
    BOB.statemachine.start()
    ELSA.statemachine.start()

    # Main Loop
    while 1:
        MASTER_CLOCK.tick()
        try:
            BaseEntity.update_all()
            POST_OFFICE.dispatch_queued()
        except GameOver:
            break
    print("Elapsed time: %d clock ticks." % MASTER_CLOCK.time())
