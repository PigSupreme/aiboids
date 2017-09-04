# -*- coding: utf-8 -*-
"""Wife Entity using simple FSM functionality"""

# for python3 compat
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import print_function

import sys
sys.path.append('..')

from random import randint

from aiboids.base_entity import BaseEntity
from aiboids.statemachine import StateMachine, State

from gamedata import Locations, MsgTypes

### GLOBAL_WIFE State Logic #########################################
WIFE_GLOBAL = State('WIFE_GLOBAL')
#    """Wife State: Global state that handles messages and chases goats!
#
#    State Transitions (these change the current state, not the global one):
#
#    * Goat in yard -> ChaseGoat
#    * on_msg MINER_HOME: if Miner is at the SHACK -> Cook Stew
#    """
@WIFE_GLOBAL.event
def on_enter(agent):
    """Check that my spouse is in the game world."""
    agent.spouse = BaseEntity.by_id(agent.spouse_id)
    try:
        print("%s : Spouse is %s" % (agent.name, agent.spouse.idstr))
    except AttributeError:
        print("%s : Looks like I'm single!" % agent.name)

@WIFE_GLOBAL.event
def on_execute(agent):
    """If I'm not in the YARD, 1 in 3 chance of a goat appearing.

    StateChange:
        Goat appears -> CHASE_GOAT

    TODO: Once we have an actual Goat, move state change logic to a more
    appropriate place (probably on_msg).
    """
    if (agent.location != Locations.YARD) and (randint(1, 3) == 1):
        agent.statemachine.change_state(CHASE_GOAT)

@WIFE_GLOBAL.event
def on_msg(agent, message):
    """If my spouse gets home, start cooking stew.

    Messages:
        * MINER_HOME: If from my spouse, change state -> COOK_STEW
    """
    if message.MSG_TYPE == MsgTypes.MINER_HOME:
        if message.SEND_ID == agent.spouse_id:
            agent.statemachine.change_state(COOK_STEW)
            return True
        else:
            return False

### DO_HOUSEWORK State Logic #########################################
DO_HOUSEWORK = State('DO_HOUSEWORK')
#    """Old West misogyny, yeehaw!
#
#    Elsa is apparently a lot tougher than her husband, since she never gets
#    tired or thirsty!
#    """
@DO_HOUSEWORK.event
def on_enter(agent):
    """Housework is done at the SHACK only; make sure I'm there."""
    if agent.location != Locations.SHACK:
        print("%s : Headin' on home..." % agent.name)
        agent.change_location(Locations.SHACK)
    print("%s : Housework ain't gonna do itself!" % agent.name)

@DO_HOUSEWORK.event
def on_execute(agent):
    """"...and sing while doing the housework, obviously."""
    if randint(0, 2):
        print("%s : Workin' round the house...tra-la-lah-la-lah..." % agent.name)

### COOK_STEW State Logic #########################################
COOK_STEW = State('COOK_STEW')
#    """More bro-gramming at it's finest, but the code is quite involved.
#
#    On entering this state, Elsa posts a delayed STEW_READY message so that
#    she knows the cooking is done. Once she receives this, she then sends an
#    immediate STEW_READY to Bob before sitting down to eat herself.
#
#    State Transitions:
#
#    * on_msg STEW_READY -> WifeEatStew
#
#    TODO: If a goat appears while Elsa is in this state, she'll revert to this
#    state after chasing the goat, and re-start dinner. Not exactly what we want.
#    """
@COOK_STEW.event
def on_enter(agent):
    """Go home and start cooking, if my spouse is still there."""
    if agent.location != Locations.SHACK:
        print("%s : Heading back to the kitchen..." % agent.name)
    # Now check if my spouse is home before I start cooking.
    if agent.spouse.location == Locations.SHACK:
        print("%s : Gonna rustle up some mighty fine stew!" % agent.name)
        # Post a message to future self to signal stew is done.
        stew_time = randint(3, 5)
        agent.send_msg(agent.me_id, MsgTypes.STEW_READY, delay=stew_time)
        agent.stew_is_on = True
    else:
        print('%s : That dang goat! I done missed mah hubby!' % agent.name)
        agent.stew_is_on = False

@COOK_STEW.event
def on_execute(agent):
    """Keep an eye on the stew, if it's still cooking.

    StateChange:
        stew_is_on == False -> revert state

    It's possible that Elsa got distracted somehow, and the stew was never
    started. If this happens, just go back to the previous state.
    """
    if agent.stew_is_on:
        print("%s : Wrassalin' with dinner..." % agent.name)
    else:
        agent.statemachine.revert_state()

@COOK_STEW.event
def on_msg(agent, message):
    """Listen for STEW_READY and notify my spouse.

    Messages:
        * STEW_READY: change state -> EAT_STEW
    """
    if message.MSG_TYPE == MsgTypes.STEW_READY:
        print("%s : Stew's ready, come an' git it!" % agent.name)
        agent.send_msg(agent.spouse_id, MsgTypes.STEW_READY)
        agent.stew_is_on = False
        agent.statemachine.change_state(EAT_STEW)
        return True
    else:
        return False

### EAT_STEW State Logic #########################################
EAT_STEW = State('EAT_STEW')
@EAT_STEW.event
def on_enter(agent):
    """Make sure I'm actually at home."""
    if agent.location != Locations.SHACK:
        print("%s : Headin' to the dinner table..." % agent.name)
        agent.change_location(Locations.SHACK)

@EAT_STEW.event
def on_execute(agent):
    """Eat and then go back to housework.

    StateChange:
        * update(1) -> DO_HOUSEWORK
    """
    print("%s : Eatin' the stew...I outdone myself this time." % agent.name)
    agent.statemachine.change_state(DO_HOUSEWORK)

CHASE_GOAT = State('CHASE_GOAT')
### CHASE_GOAT State Logic #########################################
#    """Head to the yard and shoo that goat!
#
#    Goats are stubborn critters, so there's a random chance that Elsa fails
#    to shoo the goat. But she'll keep at it until the job's done!
#
#    Goats also demand undivided attention, but Elsa has a good memory. Any
#    message received by this state will be forwarded to Elsa in the next
#    update() cycle. This means that if Bob comes home whilst Elsa's chasing a
#    goat, she'll still receive the MINER_HOME message when she's done.
#    """
@CHASE_GOAT.event
def on_enter(agent):
    """If not already in the YARD, move there to chase that goat!"""
    if agent.location != Locations.YARD:
        print("%s : Thar's a goat in mah yard!" % agent.name)
        agent.change_location(Locations.YARD)

@CHASE_GOAT.event
def on_execute(agent):
    """Attempt to shoo the goat.

    StateChange:
        Goat shooed -> revert state

    TODO: There's no actual Goat; we just fake it with Elsa's code for now.
    """
    print("%s : Shoo, ya silly goat!" % agent.name)
    # Random chance the goat will listen. Goats are stubborn
    if randint(0, 2):
        print("--FakeGoat-- : *Nom nom flowers*")
    else:
        print("--FakeGoat-- : *Scampers away*")
        agent.statemachine.revert_state()

@CHASE_GOAT.event
def on_msg(agent, message):
    """Busy chasing the goat, so forward all messages to future self."""
    agent.send_msg(agent.me_id, message.MSG_TYPE, extra=message.EXTRA, delay=1)
    return True

ENTITY_CLASS = 'Wife'
class Wife(BaseEntity):
    """Wife Elsa, scourge of the goats.

    Args:
        entity_idstr (String): The id string used by BaseEntity.

    Since there's only one Wife, everything except the BaseEntity idstr
    is hard-coded. We assume this entity is WIFE_ELSA, and her spouse is
    MINER_BOB.
    """


    def __init__(self, *args):

        super(Wife, self).__init__(*args)

        # Entities need a name and initial location
        self.name = "Wife Elsa"
        self.location = Locations.SHACK

        # For later identification, if we add additional Wives/Miners
        self.me_id = 'WIFE_ELSA'
        self.spouse_id = 'MINER_BOB'

        # Set up the StateMachine for this entity
        StateMachine(self, cur=DO_HOUSEWORK, glo=WIFE_GLOBAL)

    def update(self):
        """Just updates the StateMachine logic."""
        self.statemachine.update()

    def receive_msg(self, message):
        """Let the statemachine handle any messages."""
        self.statemachine.handle_msg(message)

    def change_location(self, newlocation):
        """Instantaneously teleport to a new location.

        Args:
            newlocation (Locations): Must match import from gamedata.py.
        """
        self.location = newlocation
