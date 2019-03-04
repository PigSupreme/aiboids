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
#    """Wife State: Global state for message handling.
#
#    State Transitions (these change the current state, not the global one):
#
#    * on_msg GOAT_IN_YARD --> CHASE_GOAT
#    * on_msg MINER_HOME --> COOK_STEW
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
def on_msg(agent, message):
    """Listen for goats eating flowers and spouse coming home."""
    # Chasing goats is more important than dinner. Bro-grammers are sad.
    if message.MSG_TYPE == MsgTypes.GOAT_IN_YARD:
        if agent.statemachine.statename != CHASE_GOAT:
            # Make note of this goat for later use
            agent.goat_target_id = message.SEND_ID
            agent.statemachine.change_state(CHASE_GOAT)
        return True
    # If my spouse is home, start cooking dinnner
    if message.MSG_TYPE == MsgTypes.MINER_HOME:
        if message.SEND_ID in (agent.spouse_id, agent.me_id):
            agent.statemachine.change_state(COOK_STEW)
            return True
    # If the stew is done but I can't react to it...
    if message.MSG_TYPE == MsgTypes.STEW_READY:
        agent.stew_is_on = False
        agent.stew_is_ruined = True
        return True

    return False

### DO_HOUSEWORK State Logic #########################################
DO_HOUSEWORK = State('DO_HOUSEWORK')
#    """Old West misogyny, yeehaw!
#
#    But Elsa is apparently a lot tougher than her husband, since she never
#    gets tired or thirsty!
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
    """"Occasionally sing while doing the housework...obviously."""
    if randint(0, 2):
        print("%s : Workin' round the house...tra-la-lah-la-lah..." % agent.name)

### COOK_STEW State Logic #########################################
COOK_STEW = State('COOK_STEW')
#    """More bro-gramming at it's finest, but the code is quite involved.
#
#    On entering this state, post a delayed STEW_READY message to self that
#    indicates cooking is done. On receipt, then send an immediate STEW_READY
#    to spouse before sitting down to eat herself.
#
#    State Transitions:
#
#    * on_msg STEW_READY --> EAT_STEW
#    """
@COOK_STEW.event
def on_enter(agent):
    """Go home and cook, if my spouse is still there."""
    if agent.location != Locations.SHACK:
        print("%s : Heading back to the kitchen..." % agent.name)
        agent.change_location(Locations.SHACK)
    # Since we might revert to this state, check if stew is on or ruined.
    if agent.stew_is_on:
        print("%s : Hope mah stew ain't burnt!" % agent.name)
        return
    if agent.stew_is_ruined:
        print("%s : Darnit, done burnt mah stew!" % agent.name)
        return

    # Otherwise, check if my spouse is home before I start cooking.
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
    """Keep an eye on the stew.

    StateChange:
        stew_is_ruined --> DO_HOUSEWORK
    """
    # It's possible that Elsa got distracted, and wasn't able to react when she
    # received the STEW_READY message. If so, the GLOBAL_WIFE state will have
    # set stew_is_on to False and stew_is_ruined to True.
    if agent.stew_is_on:
        print("%s : Wrassalin' with dinner..." % agent.name)
        return
    if agent.stew_is_ruined:
        print('%s : Better clean up this mess.' % agent.name)
        agent.statemachine.change_state(DO_HOUSEWORK)

@COOK_STEW.event
def on_msg(agent, message):
    """Listen for STEW_READY and notify my spouse.

    Messages:
        * STEW_READY: change state --> EAT_STEW
    """
    if message.MSG_TYPE == MsgTypes.STEW_READY:
        print("%s : Stew's ready, come an' git it!" % agent.name)
        agent.send_msg(agent.spouse_id, MsgTypes.STEW_READY)
        agent.stew_is_on = False
        agent.statemachine.change_state(EAT_STEW)
        return True

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
    """Quickly eat and then go back to housework.

    StateChange:
        * After one update --> DO_HOUSEWORK
    """
    print("%s : Eatin' the stew...I outdone myself this time." % agent.name)
    agent.statemachine.change_state(DO_HOUSEWORK)


### CHASE_GOAT State Logic #########################################
CHASE_GOAT = State('CHASE_GOAT')
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
    """Stare down an incoming goat, or move to the YARD and shoo it!"""
    # If already in the yard, stare down the incoming goat
    if agent.location == Locations.YARD:
        print("*** %s uses Arms Akimbo. Yul Brynner would be proud! ***")
        # It's super effective!
        agent.send_msg(agent.goat_target_id, MsgTypes.GOAT_SHOO, extra=10)
    # Otherwise, head to the YARD and prepare to shoo that goat!
    else:
        print("%s : Thar's a goat out in mah yard!" % agent.name)
        agent.change_location(Locations.YARD)

@CHASE_GOAT.event
def on_execute(agent):
    """Attempt to shoo the goat.

    StateChange:
        Target goat no longer in yard --> revert state
    """
    # Check if our target goat is still in the yard
    goat = BaseEntity.by_id(agent.goat_target_id)
    # If our target has left the yard, score a point and revert state
    if goat.location != Locations.YARD:
        print('%s : Mah flowers are safe fer now.' % agent.name)
        del agent.goat_target_id
        agent.score_shoo(1)
        agent.statemachine.revert_state()
    # Otherwise, attempt to shoo the goat.
    else:
        print("%s : Shoo, ya silly goat!" % agent.name)
        shoo_force = randint(1, 5)
        agent.send_msg(agent.goat_target_id, MsgTypes.GOAT_SHOO, shoo_force)

@CHASE_GOAT.event
def on_msg(agent, message):
    """Busy chasing the goat, so forward messages to future self if needed."""
    # STEW_READY is taken care of by the global state; make sure it gets there
    if message.MSG_TYPE == MsgTypes.STEW_READY:
        return False
    # All other message get forwarded to future self
    agent.send_msg(agent.me_id, message.MSG_TYPE, extra=message.EXTRA, delay=1)
    return True

### Information for BaseEntity ##########################################
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

        # Keeps track of cooking
        self.stew_is_on = False
        self.stew_is_ruined = False

        # For keeping score
        self.goats = 0

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

    def score_shoo(self, count):
        """Score a point for each goat chased from the yard."""
        self.goats = self.goats + count

    def final_score(self):
        """Final score = number of goats chased away."""
        return 'Chased a goat %d times.' % self.goats
