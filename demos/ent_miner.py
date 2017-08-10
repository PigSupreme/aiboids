#!/usr/bin/env python
"""Miner Entity using simple FSM functionality.
"""

# for python3 compat
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import print_function

import sys

from random import randint

sys.path.append('..')
from aiboids.base_entity import BaseEntity
from aiboids.statemachine import StateMachine, State

from gamedata import Locations, MsgTypes, GameOver


### GLOBAL_MINER State Logic ########################################
MINER_GLOBAL = State('MINER_GLOBAL')
@MINER_GLOBAL.event
def on_enter(agent):
    agent.spouse = BaseEntity.by_id(agent.spouse_id)
    try:
        print("%s : Spouse is %s" % (agent.name, agent.spouse.idstr))
    except AttributeError:
        print("%s : Looks like I'm single!" % agent.name)

@MINER_GLOBAL.event
def on_execute(agent):
    """Global miner state; increases thirst, unless we're at the SALOON."""
    if agent.location != Locations.SALOON:
        agent.thirst += 1

### DIG_IN_MINE State Logic #########################################
DIG_IN_MINE = State('DIG_IN_MINE')
@DIG_IN_MINE.event
def on_enter(agent):
    """If agent is not already in the mine, travel there."""
    if agent.location != Locations.MINE:
        print("%s : Walkin' to the gold mine..." % agent.name)
        agent.change_location(Locations.MINE)

@DIG_IN_MINE.event
def on_execute(agent):
    """Increase fatigue and dig for gold."""
    agent.add_fatigue(1)

    # Dig for gold
    gfound = randint(0,2)
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
        #agent.fsm.change_state(DrinkAtSaloon())
        return

@DIG_IN_MINE.event
def on_exit(agent):
    """Print dialog for leaving the mine."""
    print("%s : Done diggin' fer now." % agent.name)

### DEPOSIT_GOLD State Logic ########################################
DEPOSIT_GOLD = State('DEPOSIT_GOLD')
#    """Go to the bank and deposit all carried gold.
#
#    State Transitions:
#
#    * If more than 25 gold in the bank -> GameOver
#    * If work_done (enough money in bank) -> GoHomeAndRest
#    * Otherwise -> DigInMine
#    """
@DEPOSIT_GOLD.event
def on_enter(agent):
    """If agent is not at the bank, travel there."""
    if agent.location != Locations.BANK:
        print("%s : Headin' to bank, yessiree!" % agent.name)
        agent.change_location(Locations.BANK)

@DEPOSIT_GOLD.event
def on_execute(agent):
    """Deposit all the gold being carried."""
    deposit = agent.gold
    if deposit > 0:
        print("%s : Now depositin' %d gold..." % (agent.name,deposit))
        agent.change_gold(-deposit)
        agent.bank += deposit
        print("%s : Saved myself %d gold...soon'll be rich!" % (agent.name,agent.bank))
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
#
#    State Transitions:
#
#    * When no longer thirsty -> revert to previous
#    """
@DRINK_AT_SALOON.event
def on_enter(agent):
    """If not already at SALOON, go there."""
    if agent.location != Locations.SALOON:
        print("%s : Headin' to the saloon fer a drink..." % agent.name)
        agent.change_location(Locations.SALOON)

@DRINK_AT_SALOON.event
def on_execute(agent):
    """Have a drink."""
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
#    """Go home and rest.
#
#    When Miner Bob enters this state, he sends Elsa a message to start cooking
#    the stew. He's apparently impatient or a workaholic, because he will go
#    back to the mine once fully rested, even if he's not eaten yet. Poor Elsa!
#
#    State Transitions:
#
#    * Once fully rested -> DigInMine
#    * If stew is ready and is still in SHACK -> MinerEatStew
#    """

@REST_AT_HOME.event
def on_enter(agent):
    # If not at SHACK, go there and tell the wife we're home
    if agent.location != Locations.SHACK:
        print("%s : Day's a finished, headin' on home!" % agent.name)
        agent.change_location(Locations.SHACK)
        agent.send_msg(agent.spouse_id, 'MsgTypes.MINER_HOME')

@REST_AT_HOME.event
def on_execute(agent):
    # Take a nap if not fully rested
    if agent.fatigue > 0:
        print("%s : Zzzzzz...." % agent.name)
        agent.remove_fatigue(1)
    else:
        print("%s : Done restin' fer now, back to work!" % agent.name)
        agent.statemachine.change_state(DIG_IN_MINE)

@REST_AT_HOME.event
def on_msg(agent, message):
    # If stew's ready, wake up and eat
    if message.MSG_TYPE == MsgTypes.STEW_READY:
        if message.SEND_ID == agent.spouse_id and agent.location == Locations.SHACK:
            print("%s : I hears ya', lil' lady..." % agent.name)
            agent.statemachine.change_state(EAT_STEW)
            return True

    return False

### EAT_STEW State Logic ##########################################
EAT_STEW = State('EAT_STEW')
#    """Eat that tasty stew, and thank yer lovely wife!
#
#    Food removes fatigue, of course.
#
#    State Transitions:
#
#    * After a single execute() to eat stew -> revert to previous
#    """
@EAT_STEW.event
def on_enter(agent):
    if agent.location != Locations.SHACK:
        print("%s : Better git home fer dinner..." % agent.name)
        agent.change_location(Locations.SHACK)

@EAT_STEW.event
def on_execute(agent):
    print("%s : That's some might fine stew...thank ya much, Elsa!" % agent.name)
    agent.remove_fatigue(5)
    agent.statemachine.revert_state()

@EAT_STEW.event
def on_exit(agent):
    print("%s : Now where was I...?" % agent.name)

class Miner(BaseEntity):
    """Miner Bob.

    Note: The constructor doesn't take any actual args, but this syntax is
    needed to call the __init__ method of the superclass. I'm not sure that
    we need to do so here, but it will be a useful reminder for later.
    """

    def __init__(self, *args):
        super(Miner, self).__init__(*args)
        self.name = "Miner Bob"
        self.location = Locations.SHACK
        self.gold = 0
        self.bank = 0
        self.work_goal = 10
        self.thirst = 0
        self.fatigue = 0

        # For later identification, if we add additional Wives/Miners
        self.me_id = 'MINER_BOB'
        self.spouse_id = 'WIFE_ELSA'

        # Set up the FSM for this entity
        StateMachine(self, cur=DIG_IN_MINE, glo=MINER_GLOBAL)

    def update(self):
        """Just updates the StateMachine logic."""
        self.statemachine.update()

    def receive_msg(self,message):
        # Let the FSM handle any messages
        self.statemachine.handle_msg(message)

    def change_location(self,newlocation):
        """Instantaneously teleport to a new location.

        Parameters
        ----------
        newlocation: LOCATION_CONSTANT
            Enumerated location, imported from gamedata.py
        """
        self.location = newlocation

    def change_gold(self,amount):
        """Add/subtract the amount of gold currently carried

        Parameters
        ----------
        amount: int
            Amount of gold to add (or subtract, if negative)
        """
        self.gold += amount

    def pockets_full(self):
        """Queries whether this entity is carrying enough gold."""
        return (self.gold >= 3)

    def add_fatigue(self,amount=1):
        """Increases the current fatigue of this entity."""
        self.fatigue += amount

    def remove_fatigue(self,amount=1):
        """Remove fatigue from this entity, but not below zero."""
        self.fatigue -= amount
        if self.fatigue < 0:
            self.fatigue = 0

    def is_thirsty(self):
        """Queries if this entity has too much current thirst."""
        return (self.thirst > 7)

    def remove_thirst(self,amount):
        """Remove thirst from this entity, but not below zero."""
        self.thirst -= amount
        if self.thirst < 0:
            self.thirst = 0

    def work_done(self):
        """Check if we're done working for the day."""
        if self.bank >= self.work_goal:
            self.work_goal += 10
            return True
        else:
            return False
