#!/usr/bin/env python
"""Miner Entity using simple FSM functionality."""

# for python3 compat
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import print_function

import sys
from random import randint

# Non-system imports here:
sys.path.append('..')
from aiboids.base_entity import BaseEntity
from aiboids.statemachine import StateMachine, State
from gamedata import Locations, MsgTypes, GameOver

### GLOBAL_MINER State Logic ########################################
MINER_GLOBAL = State('MINER_GLOBAL')
@MINER_GLOBAL.event
def on_enter(agent):
    """Check that my spouse is in the game world."""
    agent.spouse = BaseEntity.by_id(agent.spouse_id)
    try:
        print("%s : Spouse is %s" % (agent.name, agent.spouse.idstr))
    except AttributeError:
        print("%s : Looks like I'm single!" % agent.name)

@MINER_GLOBAL.event
def on_execute(agent):
    """Increases thirst, unless I'm at the SALOON."""
    if agent.location != Locations.SALOON:
        agent.thirst += 1

### DIG_IN_MINE State Logic #########################################
DIG_IN_MINE = State('DIG_IN_MINE')
@DIG_IN_MINE.event
def on_enter(agent):
    """If not already in the mine, travel there."""
    if agent.location != Locations.MINE:
        print("%s : Walkin' to the gold mine..." % agent.name)
        agent.change_location(Locations.MINE)

@DIG_IN_MINE.event
def on_execute(agent):
    """Increase fatigue and dig for gold.

    StateChange:
        * Pockets are Full --> DEPOSIT GOLD
        * Thirsty --> DRINK_AT_SALOON
    """
    agent.add_fatigue(1)

    # Dig for gold
    gfound = randint(0, 2)
    chatter = [
        "Keep on a-diggin'...",
        "Found me a gold nugget!",
        "Found me two nuggets, whaddaya know!"
        ][gfound]
    print("%s : %s " % (agent.name, chatter))
    agent.change_gold(gfound)

    # If pockets are full, go visit the bank
    if agent.pockets_full():
        agent.statemachine.change_state(DEPOSIT_GOLD)
        return # So that we don't try another state change below

    # If thirsty, go visit the saloon
    if agent.is_thirsty():
        agent.statemachine.change_state(DRINK_AT_SALOON)
        return

@DIG_IN_MINE.event
def on_exit(agent):
    """Print dialog for leaving the mine."""
    print("%s : Done diggin' fer now." % agent.name)

### DEPOSIT_GOLD State Logic ########################################
DEPOSIT_GOLD = State('DEPOSIT_GOLD')
@DEPOSIT_GOLD.event
def on_enter(agent):
    """If I'm not at the bank, travel there."""
    if agent.location != Locations.BANK:
        print("%s : Headin' to bank, yessiree!" % agent.name)
        agent.change_location(Locations.BANK)

@DEPOSIT_GOLD.event
def on_execute(agent):
    """Deposit all the gold being carried, and check for victory.

    StateChange:
    * If work_done (enough money in bank) --> REST_AT_HOME
    * Otherwise --> DIG_IN_MINE
    """
    deposit = agent.gold
    if deposit > 0:
        print("%s : Now depositin' %d gold..." % (agent.name, deposit))
        agent.change_gold(-deposit)
        agent.bank += deposit
        print("%s : Saved myself %d gold...soon'll be rich!" % (agent.name, agent.bank))
        if agent.bank > 25:
            print("%s : Whee, doggy! A winner is y'all!" % agent.name)
            raise GameOver
        # If wealthy enough, go home and sleep
        if agent.work_done():
            agent.statemachine.change_state(REST_AT_HOME)
            return
        # Otherwise, back to work!
        agent.statemachine.change_state(DIG_IN_MINE)

@DEPOSIT_GOLD.event
def on_exit(agent):
    """Print dialog for leaving the bank."""
    print("%s : Leavin' the bank..." %  agent.name)

### DRINK_AT_SALOON State Logic ########################################
DRINK_AT_SALOON = State('DRINK_AT_SALOON')
#    """Go to the saloon and drink until thirst is quenched

@DRINK_AT_SALOON.event
def on_enter(agent):
    """If not already at SALOON, go there."""
    if agent.location != Locations.SALOON:
        print("%s : Headin' to the saloon fer a drink..." % agent.name)
        agent.change_location(Locations.SALOON)

@DRINK_AT_SALOON.event
def on_execute(agent):
    """Drink until I've quenched my thirst.

    StateChange:
    * When no longer thirsty -> revert to previous
    """
    print("%s : Havin' a whiskey...mighty refreshin'!" % agent.name)
    agent.remove_thirst(5)
    #If no longer thirsty, go back to whatever we were doin'
    if not agent.is_thirsty():
        agent.statemachine.revert_state()
        return

@DRINK_AT_SALOON.event
def on_exit(agent):
    """Print dialog for leaving the saloon."""
    print("%s : Leavin' the saloon fer now..." % agent.name)

### REST_AT_HOME State Logic ##########################################
REST_AT_HOME = State('REST_AT_HOME')
#    When Miner Bob enters this state, he sends Elsa a message to start cooking
#    the stew. He's apparently impatient or a workaholic, because he will go
#    back to the mine once fully rested, even if he's not eaten yet. Poor Elsa!
@REST_AT_HOME.event
def on_enter(agent):
    """If not at SHACK, go there. Tell my spouse I'm home."""
    if agent.location != Locations.SHACK:
        print("%s : Day's a finished, headin' on home!" % agent.name)
        agent.change_location(Locations.SHACK)
        agent.send_msg(agent.spouse_id, MsgTypes.MINER_HOME)

@REST_AT_HOME.event
def on_execute(agent):
    """Sleep until rested or woken for dinner.

    StateChange:
    * When fully rested -> DIG_IN_MINE
    """
    # Take a nap if not fully rested
    if agent.fatigue > 0:
        print("%s : Zzzzzz...." % agent.name)
        agent.remove_fatigue(1)
    else:
        print("%s : Done restin' fer now, back to work!" % agent.name)
        agent.statemachine.change_state(DIG_IN_MINE)

@REST_AT_HOME.event
def on_msg(agent, message):
    """Wake up when dinner is ready.

    Messages:
    * STEW_READY: If from my spouse, change state -> EAT_STEW
    """
    if message.MSG_TYPE == MsgTypes.STEW_READY:
        if message.SEND_ID == agent.spouse_id and agent.location == Locations.SHACK:
            print("%s : I hears ya', lil' lady..." % agent.name)
            agent.statemachine.change_state(EAT_STEW)
            return True
    return False

### EAT_STEW State Logic ##########################################
EAT_STEW = State('EAT_STEW')
@EAT_STEW.event
def on_enter(agent):
    """It doesn't matter where you get hungry, as long as you go home for dinner."""
    if agent.location != Locations.SHACK:
        print("%s : Better git home fer dinner..." % agent.name)
        agent.change_location(Locations.SHACK)

@EAT_STEW.event
def on_execute(agent):
    """Eat that tasty stew and recover some fatigue.

    StateChange:
        update(1) --> revert to previous
    """
    print("%s : That's some might fine stew...thank ya much, Elsa!" % agent.name)
    agent.remove_fatigue(5)
    agent.statemachine.revert_state()

@EAT_STEW.event
def on_exit(agent):
    """Print after-dinner dialog."""
    print("%s : Now where was I...?" % agent.name)

# Used by gamedata.py for automatic import.
ENTITY_CLASS = 'Miner'
class Miner(BaseEntity):
    """Miner Bob, the only miner in the world!

    Args:
        entity_idstr (String): The id string used by BaseEntity.

    Since there's only one Miner, everything except the BaseEntity idstr
    is hard-coded. We assume this entity is MINER_BOB, and his spouse is
    WIFE_ELSA.
    """

    def __init__(self, entity_idstr):

        super(Miner, self).__init__(entity_idstr)
        self.name = "Miner Bob"
        self.location = Locations.SHACK
        self.gold = 0
        self.bank = 0
        self.work_goal = 10
        self.thirst = 0
        self.fatigue = 0

        # For later identification, if we add additional Wives/Miners
        self.me_id = entity_idstr
        self.spouse_id = 'WIFE_ELSA'

        # Set up the FSM for this entity
        StateMachine(self, cur=DIG_IN_MINE, glo=MINER_GLOBAL)

    def update(self):
        """Just updates the StateMachine logic."""
        self.statemachine.update()

    def receive_msg(self, message):
        """Let our statemachine handle any messages."""
        self.statemachine.handle_msg(message)

    def change_location(self, newlocation):
        """Instantaneously teleport to a new location.

        Args:
            newlocation (Locations): Must match import from gamedata.py.
        """
        if newlocation in Locations:
            self.location = newlocation
        else:
            raise ValueError("Location.%s is not defined." % newlocation)

    def change_gold(self, amount):
        """Add/subtract the amount of gold currently carried

        Args:
            amount: How much gold to add (or subtract, if negative)
        """
        self.gold += amount

    def pockets_full(self):
        """True if this Miner is carrying enough gold (3+ nuggets)."""
        return self.gold >= 3

    def add_fatigue(self, amount=1):
        """Increases our current fatigue."""
        self.fatigue += amount

    def remove_fatigue(self, amount=1):
        """Remove fatigue, but don't let it go below zero."""
        self.fatigue -= amount
        if self.fatigue < 0:
            self.fatigue = 0

    def is_thirsty(self):
        """True if this Miner is thirsty."""
        return self.thirst > 7

    def remove_thirst(self, amount):
        """Remove thirst, but don't let it go below zero."""
        self.thirst -= amount
        if self.thirst < 0:
            self.thirst = 0

    def work_done(self):
        """Check if we've met our mining goal for now."""
        if self.bank >= self.work_goal:
            self.work_goal += 10
            return True
        else:
            return False

    def final_score(self):
        return 'Deposited %d gold nuggets.' % self.bank
