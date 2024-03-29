#!/usr/bin/env python
"""Game-wide constants for WestWorld3 demo.

Game Entities
-------------
* MINER_BOB: Miner Bob, Elsa's husband
* WIFE_ELSA: Wife Elsa, scourge of wandering goats
* BILLY: The goat (now a real GameEntity!)

Locations
---------
* SHACK: Bob and Elsa's humble home
* MINE: Gold mine. Dig for nuggets here!
* BANK: A bank, duh. Deposit nuggets here!
* SALOON: Quench yer thirst here!
* YARD: Frequently invaded by flower-eating goats!
* FIELDS: Thar be goats in them fields!

Message Types
-------------
* MINER_HOME: Bob sends this when he comes home from digging.
* STEW_READY: Elsa sends this when she's finished cooking.
* GOAT_IN_YARD: Send this to Elsa when Billy enters the yard.
* GOAT_SHOO: Elsa sends to Billy to attempt to shoo (extra=shoo_force)
* GOAT_OUT: Reserved for future use.
"""

from enum import Enum

# Enumeration of characters {name: sourcefile}
Characters = {'MINER_BOB' : 'ent_miner',
              'WIFE_ELSA' : 'ent_wife',
              'GOAT_BILLY' : 'ent_goat'
              }

# Enumeration of locations
Locations = Enum('Locations', 'SHACK MINE BANK SALOON YARD FIELDS')

# Enumeration of message types
MsgTypes = Enum('MsgTypes', 'MINER_HOME STEW_READY GOAT_IN_YARD GOAT_SHOO GOAT_OUT')

class GameOver(Exception):
    """Raise this exception to end the game."""
    pass

if __name__ == "__main__":
    print(__doc__)
