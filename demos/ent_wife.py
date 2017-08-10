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
    """Find that hubby o' mine!"""

    agent.spouse = BaseEntity.by_id(agent.spouse_id)
    try:
        print("%s : Spouse is %s" % (agent.name, agent.spouse.idstr))
    except AttributeError:
        print("%s : Looks like I'm single!" % agent.name)

@WIFE_GLOBAL.event
def on_execute(agent):
    # If not in the YARD, random chance of goat appearing
    if (agent.location != Locations.YARD) and (randint(1, 3) == 1):
        agent.statemachine.change_state(CHASE_GOAT)

@WIFE_GLOBAL.event
def on_msg(agent, message):
    if message.MSG_TYPE == 'MsgTypes.MINER_HOME':
        # Ignore message if it's not from my spouse
        if message.SEND_ID == agent.spouse_id:
            agent.statemachine.change_state(COOK_STEW)
            return True
        else:
            return False

### DO_HOUSEWORK State Logic #########################################
DO_HOUSEWORK = State('DO_HOUSEWORK')
#    """Old West misogyny, yeehaw!
#
#    Note
#    ----
#
#    Elsa is apparently a lot tougher than her husband, since she never gets
#    tired or thirsty! We should probably give her some more interesting things
#    to do...a good exercise in FSM design/coding!
#
#    This state has no transitions; those are handled by GlobalWifeState.
#    """

@DO_HOUSEWORK.event
def on_enter(agent):
    # Housework is done at the SHACK only.
    if agent.location != Locations.SHACK:
        print("%s : Headin' on home..." % agent.name)
        agent.change_location(Locations.SHACK)
    print("%s : Housework ain't gonna do itself!" % agent.name)

@DO_HOUSEWORK.event
def on_execute(agent):
    # ...and she sings while doing the housework, obviously.
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
    if agent.location != Locations.SHACK:
        print("%s : Heading back to the kitchen..." % agent.name)
    # Check if my spouse is home before I start cooking.
    if agent.spouse.location == Locations.SHACK:
        print("%s : Gonna rustle up some mighty fine stew!" % agent.name)
        # Post a message to future self to signal stew is done.
        stew_time = randint(3, 5)
        agent.send_msg(agent.me_id, MsgTypes.STEW_READY, delay=stew_time)
    else:
        print('%s : That dang goat! I done missed mah hubby!' % agent.name)

@COOK_STEW.event
def on_execute(agent):
    print("%s : Wrassalin' with dinner..." % agent.name)

@COOK_STEW.event
def on_msg(agent, message):
    if message.MSG_TYPE == MsgTypes.STEW_READY:
        print("%s : Stew's ready, come an' git it!" % agent.name)
        agent.send_msg(agent.spouse_id, MsgTypes.STEW_READY)
        agent.statemachine.change_state(EAT_STEW)
        return True
    else:
        return False

### EAT_STEW State Logic #########################################
EAT_STEW = State('EAT_STEW')
#    """Eat that tasty stew!
#
#    State Transitions:
#
#    * After one execute() to eat stew -> DoHouseWork
#    """
@EAT_STEW.event
def on_enter(agent):
    if agent.location != Locations.SHACK:
        print("%s : Headin' to the dinner table..." % agent.name)
        agent.change_location(Locations.SHACK)

@EAT_STEW.event
def on_execute(agent):
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
#
#    State Transitions:
#
#    * Successfully shoos the goat -> revert to previous
#    """
@CHASE_GOAT.event
def on_enter(agent):
    # If not already in the yard, move there and chase that goat!
    if agent.location != Locations.YARD:
        print("%s : Thar's a goat in mah yard!" % agent.name)
        agent.change_location(Locations.YARD)

@CHASE_GOAT.event
def on_execute(agent):
    print("%s : Shoo, ya silly goat!" % agent.name)
    # Random chance the goat will listen. Goats are stubborn
    if randint(0, 2):
        print("--FakeGoat-- : *Nom nom flowers*")
    else:
        print("--FakeGoat-- : *Scampers away*")
        agent.statemachine.revert_state()

@CHASE_GOAT.event
def on_msg(agent, message):
    #Busy chasing the goat, so forward messages to future self.
    agent.send_msg(agent.me_id, message.MSG_TYPE, extra=message.EXTRA, delay=1)
    return True

ENTITY_CLASS = 'Wife'
class Wife(BaseEntity):
    """Wife Elsa, scourge of the goats.

    Note: The constructor doesn't take any actual args, but this syntax is
    needed to call the __init__ method of the superclass. I'm not sure that
    we need to do so here, but it will be a useful reminder for later.
    """

    def __init__(self, *args):
        # Calls BaseEntity.__init__ to set-up basic functionality.
        super(Wife, self).__init__(*args)

        # Entities need a name and initial location
        self.name = "Wife Elsa"
        self.location = Locations.SHACK

        # For later identification, if we add additional Wives/Miners
        self.me_id = 'WIFE_ELSA'
        self.spouse_id = 'MINER_BOB'

        # Set up the FSM for this entity
        StateMachine(self, cur=DO_HOUSEWORK, glo=WIFE_GLOBAL)

    def update(self):
        """Just updates the StateMachine logic."""
        self.statemachine.update()

    def receive_msg(self, message):
        """Let the statemachine handle any messages."""
        self.statemachine.handle_msg(message)

    def change_location(self, newlocation):
        """Instantaneously teleport to a new location.

        Parameters
        ----------
        newlocation: LOCATION_CONSTANT
            Enumerated location, imported from gamedata.py
        """
        self.location = newlocation
