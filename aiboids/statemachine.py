#!/usr/bin/env python
"""Module containing basic UML State Machine functionality.

All states should be derived from the State class, see its documentation.

Use STATE_NONE as a concrete null state. We need only a single instance.

An agent can be given state machine functionality as follows:

* fsm = StateMachine(agent)
** This automatically sets agent.statemachine = fsm
** StateMachine() allows optional keywords cur, glo, pre to set states, or...
** ...use fsm.set_state(current); with optional kewords for glo and pre.
* Call agent.statemachine.update() to execute the agent's current state logic. 
* Call agent.fsm.handle_msg(message) to let the states deal with messages.
"""

from __future__ import print_function

class State(object):
    """Provides the necessary framework for UML state logic (with events).

    Instances of this class are set up with a set of dummy hooks for the
    state machine logic: on_enter, on_execute, on_exit, and on_msg. Dummy
    versions do nothing but can be overridden on a per-instance basis by
    setting the appropriate attribute after instantiation.
    
    Each enter/execute/exit method must be a callable with a single positional
    parameter: the agent using this State's logic. on_msg needs an additional
    second positional parameter to hold the incoming message.
    """
    def __init__(self, name=None):
        self.on_enter = State.on_dummy
        self.on_execute = State.on_dummy
        self.on_exit = State.on_dummy
        self.on_msg = State.msg_dummy
        if name is not None:
            self.name = str(name)

    # Dummy placeholder for on{enter/execute/exit} methods.
    @staticmethod
    def on_dummy(dummy_agent):
        pass
    
    # Dummy placeholder for on_msg.
    @staticmethod
    def msg_dummy(dummy_agent, dummy_msg):
        return False

#: A sample null state. Prints a warning when we try to call its on_enter().
STATE_NONE = State()
STATE_NONE.on_enter = lambda agent: print("WARNING: Agent %s entered STATE_NONE" % agent)

class StateMachine(object):
    """UML-Style State Machine with event/messaging capability.

    Parameters
    ----------
    owner: BaseEntity
        The entity using this StateMachine
    cur, glo, pre: State, optional
        Keyword arguments for setting the current, global, or previous state

    Instantiating this class will automatically set owner.statemachine to the
    new instance. Omitting any of the cur/glo/pre keyword arguments will set
    the corresponding states to None; these can also be assigned or changed
    later using StateMachine.set_state().
    """
    def __init__(self, owner, **kwargs):
        self.owner = owner
        owner.statemachine = self
        for stype in ('cur', 'glo', 'pre'):
            if stype in kwargs:
                setattr(self, stype+'_state', kwargs[stype])
            else:
                setattr(self, stype+'_state', None)

    @property
    def statename(self):
        """Get the name of this state machine's current state."""
        if self.cur_state:
            return self.cur_state.name

    def set_state(self, *args, **kwargs):
        """Manually set owner's states without triggering state change logic.

        This method is primarily meant for initialization of the StateMachine.
        If called with a single parameter, we assume this to be the desired
        "current" state, and leave the global/previous states unchanged; these
        default to None when the StateMachine is created.
        
        The "global" and "previous" states can be set using keyword arguments
        'glo' and 'pre', respectively.       
        """
        # If present, assume first positional argument is a current state
        if args is not None:
            self.cur_state = args[0]
        if 'glo' in kwargs.keys():
            self.glo_state = kwargs['glo']
        if 'pre' in kwargs.keys():
            self.pre_state = kwargs['pre']

    def start(self):
        """Start the FSM by executing global & current state's enter() methods."""
        if self.glo_state:
            self.glo_state.on_enter(self.owner)
        if self.cur_state:
            self.cur_state.on_enter(self.owner)

    def update(self):
        """Execute the owner's global state (if any), then current state."""
        # First execute a global state if it exists
        if self.glo_state:
            self.glo_state.on_execute(self.owner)
        # Now execute the current state (which should exist, but check anyway)
        if self.cur_state:
            self.cur_state.on_execute(self.owner)

    def change_state(self, newstate):
        """Switches to a new state, calling appropriate exit/enter methods.

        Parameters
        ----------
        newstate: State
            The StateMachine will switch to this state.

        Assuming the current and newstates are both valid, this does:
        * Call the exit method of the current state.
        * Change to the newstate.
        * Call the enter method of the newstate.
        """
        if self.cur_state and newstate:
            self.pre_state = self.cur_state
            self.cur_state.on_exit(self.owner)
            self.cur_state = newstate
            self.cur_state.on_enter(self.owner)

    def revert_state(self):
        """Reverts owner to its previous state, if one exists."""
        if self.pre_state:
            self.change_state(self.pre_state)

    def handle_msg(self, message):
        """Lets the StateMachine attempt to handle a received message.

        The message is first passed to the current state, which tries to
        handle it. If the current state's on_msg() returns False, the message
        is then passed up to the global state.
        
        There is no particular structure required of the message; it will just
        be passed along to the on_msg() function of current/global States.

        Returns
        -------
        bool
            True if the message was handled by either the current or global
            state; False otherwise.
        """
        # First let the current state try to handle this message...
        if self.cur_state and self.cur_state.on_msg(self.owner, message):
            return True
        # ...but if handling was not done, send to the global state.
        if self.glo_state and self.glo_state.on_msg(self.owner, message):
            return True
        else:
            return False

def sample_run():
    """Demo using the classic turnstile two-state FSM."""
    class Turnstile():
        pass
    
    turnstile_locked = State('STATE_LOCKED')

    def locked_msg(agent, message):
        if message == 'COIN':
            print('Coin received, unlocking!')
            agent.statemachine.change_state(turnstile_unlocked)
        if message == 'PUSH':
            print('Denied!')
    turnstile_locked.on_msg = locked_msg

    turnstile_unlocked = State('STATE_UNLOCKED')
    def unlocked_msg(agent, message):
        if message == 'PUSH':
            print('There it goes!')
            agent.statemachine.change_state(turnstile_locked)
    turnstile_unlocked.on_msg = unlocked_msg
    
    thing = Turnstile()
    fsm = StateMachine(thing)
    fsm.set_state(turnstile_locked, glo=STATE_NONE)
    
    fsm.start()
    for i in range(1,20):
        print('\nUpdate %d: Starting state is %s' % (i,thing.statemachine.statename))
        if i%5 == 0:
            print('Inserting coin...')
            fsm.handle_msg('COIN')
        
        if i%3 == 0:
            print('Trying to push...')
            fsm.handle_msg('PUSH')
        fsm.update()

if __name__ == "__main__":
    sample_run()