#!/usr/bin/env python
"""Game-wide constants.

Game Entities
-------------
* MINER_BOB: Miner Bob, Elsa's husband
* WIFE_ELSA: Wife Elsa, scourge of wandering goats
* BILLY: The goat

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
"""

from enum import Enum

CharTypes = Enum('CharTypes', 'Miner Wife Goat')

# Enumeration of characters
Characters = {'MINER_BOB' : CharTypes.Miner,
              'WIFE_ELSA' : CharTypes.Wife,
              'BILLY' : CharTypes.Goat}

# Enumeration of locations
Locations = Enum('Locations', 'SHACK MINE BANK SALOON YARD FIELDS')

# Enumeration of message types
MsgTypes = Enum('MsgTypes', 'MINER_HOME STEW_READY')

class GameOver(Exception):
    """Raise this exception to end the game."""
    pass

if __name__ == "__main__":
    print(__doc__)
