# -*- coding: utf-8 -*-
"""Goat Entity using simple FSM functionality"""

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
GOAT_GLOBAL = State('GOAT_GLOBAL')

@GOAT_GLOBAL.event
def on_enter(agent):
    """Check that I have somebody's flowers to eat."""
    agent.pester = BaseEntity.by_id(agent.pester_id)
    try:
        print("%s : I will eat the flowers of %s" % (agent.name, agent.pester.idstr))
    except AttributeError:
        print("%s : Oh noes, goat will starve!" % agent.name)

@GOAT_GLOBAL.event
def on_execute(agent):
    """Bleat every so often, because that's what goats do."""
    if randint(0, 4):
        print("%s : Baaaaah!" % agent.name)


### WANDER_FIELDS State Logic #########################################
WANDER_FIELDS = State('WANDER_FIELDS')

@WANDER_FIELDS.event
def on_enter(agent):
    """If not in the FIELDS, go there and wander for a bit."""
    if agent.location != Locations.FIELDS:
        agent.change_location(Locations.FIELDS)
    agent.wander_countdown = randint(3, 6)

@WANDER_FIELDS.event
def on_execute(agent):
    """Wander aimlessly for some time.

    StateChange:
        Wander countdown complete --> EAT_FLOWERS
    """
    agent.wander_countdown = agent.wander_countdown - 1
    if agent.wander_countdown <= 0:
        agent.statemachine.change_state(EAT_FLOWERS)

### EAT_FLOWERS State Logic #########################################
EAT_FLOWERS = State('EAT_FLOWERS')

@EAT_FLOWERS.event
def on_enter(agent):
    """If not in the YARD, go there and let Elsa know."""
    if agent.location != Locations.YARD:
        agent.change_location(Locations.YARD)
    agent.send_msg(agent.pester_id, MsgTypes.GOAT_IN_YARD)

@EAT_FLOWERS.event
def on_execute(agent):
    """Eat flowers until we're shooed away."""
    # Random number of flowers each time
    agent.eat_flowers(randint(1, 3))

@EAT_FLOWERS.event
def on_msg(agent, message):
    """Possibly be shooed by Elsa, but goats are stubborn.

    Messages:
        * GOAT_SHOO: If sufficient force, change state --> WANDER_FIELDS
    """
    if message.MSG_TYPE == MsgTypes.GOAT_SHOO:
        # If the shoo is stronger than we are stubborn, go back to the FIELDS
        if message.EXTRA > agent.stubbornness:
            print('%s : *Scampers away*' % agent.name)
            agent.statemachine.change_state(WANDER_FIELDS)


ENTITY_CLASS = 'Goat'
class Goat(BaseEntity):
    """An actual goat, not just a placeholder for some non-goat.

    Args:
        entity_idstr (String): The id string used by BaseEntity.

    Since there's only one Goat, everything except the BaseEntity idstr
    is hard-coded. We assume this entity is GOAT_BILLY, and will pester
    WIFE_ELSA.
    """

    def __init__(self, *args):
        # Calls BaseEntity.__init__ to set-up basic functionality.
        super(Goat, self).__init__(*args)

        # Entities need a name and initial location
        self.name = "Billy the Goat" # Will we also have Billy the Kid?
        self.location = Locations.FIELDS

        # For later identification, because RAWR MOAR GOATS!!!!
        self.me_id = 'GOAT_BILLY'
        self.pester_id = 'WIFE_ELSA'

        # Resistance to being shooed from the YARD
        self.stubbornness = 2

        # Keep track of what we've eaten
        self.flowers = 0

        # Set up the StateMachine for this entity
        StateMachine(self, cur=WANDER_FIELDS, glo=GOAT_GLOBAL)

    def update(self):
        """To be called by the EntityManager each update."""
        # Update the FSM logic, and nothing else for now.
        self.statemachine.update()

    def receive_msg(self, message):
        """Used by the EntityManage for basic messaging."""
        # Let the FSM handle any messages
        self.statemachine.handle_msg(message)

    def change_location(self, newlocation):
        """Instantaneously teleport to a new location.

        Args:
            newlocation (Locations): Must match import from gamedata.py.
        """
        self.location = newlocation

    def eat_flowers(self, count):
        """Nom nom flowers."""
        noms = "Nom" + (count-1)*" nom"
        print('%s : *%s flowers*' % (self.name, noms))
        self.flowers = self.flowers + count

    def final_score(self):
        """Final score = number of flowers nommed."""
        return 'Ate %d flowers.' % self.flowers
