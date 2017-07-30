#!/usr/bin/env python
"""Module containing basic UML State Machine functionality.

Actual states should be instances of the State class; its documentation shows
how to define the necessary state logic methods. We provide STATE_NONE as a
simple example; this can also be used for an concrete initial state.

The object `agent` can be given state machine functionality as follows:

>>> fsm = StateMachine(agent)  # which also sets agent.statemachine = fsm
>>> # Use optional keywords cur, glo, pre to set states above, or ...
>>> fsm.set_state(current)     # with optional kewords glo and pre.
>>> # ...
>>> agent.statemachine.update()  # to execute the current state logic.
>>> # ...
>>> agent.statemachine.handle_msg(msg) # to send msg via the statemachine
"""

from __future__ import print_function

class State(object):
    """Provides the necessary framework for UML state logic (with events).

    Instances of this class are initiated with a set of default hooks for the
    state machine logic: on_enter, on_execute, on_exit, and on_msg.

    The defaults do nothing, but can be overridden on a per-instance basis.
    Each enter/execute/exit method must be a callable with a single positional
    parameter: the agent using this State's logic. The on_msg method needs an
    additional second positional parameter to hold the incoming message.

    After calling statename = State(), the function decorator @statename.event
    is define to easily replace the default hook methods above. For example:

    >>> STATE_NONE = State()
    >>> @STATE_NONE.event
    >>> def on_enter(agent):
    >>>     print("WARNING: Agent %s entered STATE_NONE" % agent)

    will set STATE_NONE.on_enter to the newly-defined function. Future plans
    are to use the decorator syntax to generate automatic state documentation,
    but we're not quite there yet.
    """
    def __init__(self, name="SomeState"):
        self.on_enter = State.on_dummy
        self.on_execute = State.on_dummy
        self.on_exit = State.on_dummy
        self.on_msg = State.msg_dummy
        self.name = str(name)
        self.register_state(self, name)

    @classmethod
    # For keeping track of defined states
    def register_state(cls, state, statename):
        if not hasattr(cls, 'statedir'):
            cls.statedir = {}
        cls.statedir[state] = statename

    # Dummy placeholder for on{enter/execute/exit} methods.
    @staticmethod
    def on_dummy(dummy_agent):
        pass

    # Dummy placeholder for on_msg.
    @staticmethod
    def msg_dummy(dummy_agent, dummy_msg):
        return False

    def event(self, eventhook):
        if eventhook.__name__ in ('on_enter','on_execute','on_exit','on_msg'):
            setattr(self, eventhook.__name__, eventhook)
            # Note how we can grab the docstring from the decorated function!
            if eventhook.__doc__:
                print(str(self.name) +"." + eventhook.__name__ +" : "+ eventhook.__doc__)
                # TODO: We can change the docstring, but need to figure out how autodocs can find it.
                setattr(eventhook, "__doc__", "GOAT!")
            return None
        else:
            raise ValueError('State event %s not recognized' % eventhook.__name__)

################ A very simple State #####################
#: Used as a default null state by the StateMachine class.
STATE_NONE = State()

@STATE_NONE.event
def on_enter(agent):
    """The decorator will find this docstring!"""
    print("WARNING: Agent %s entered STATE_NONE" % agent)
##########################################################

class StateMachine(object):
    """UML-Style State Machine with event/messaging capability.

    Args:
        owner (BaseEntity): The entity that will use this StateMachine.

    Keyword Args:
        cur (State): Sets the current state.
        glo (State): Sets the global state.
        pre (State): Sets the previous state.

    Instantiating this class will automatically set owner.statemachine to the
    new instance. Omitting any of the keyword arguments will set their
    corresponding states to None; these can also be assigned/changed later
    using StateMachine.set_state().
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

        Args:
            newstate (State): The StateMachine will switch to this state.

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

        Returns:
            bool: True if the message was reported as being handled by either
            the current or global state; False otherwise.
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

    @turnstile_locked.event
    def on_msg(agent, message):
        """COIN: Change state to unlocked. PUSH: Print denial message."""
        if message == 'COIN':
            print('Coin received, unlocking!')
            agent.statemachine.change_state(turnstile_unlocked)
        if message == 'PUSH':
            print('Denied!')

    turnstile_unlocked = State('STATE_UNLOCKED')
    @turnstile_unlocked.event
    def on_msg(agent, message):
        if message == 'PUSH':
            print('There it goes!')
            agent.statemachine.change_state(turnstile_locked)

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
    #sample_run()
    """Demo using the classic turnstile two-state FSM."""
    class Turnstile():
        pass

    turnstile_locked = State('STATE_LOCKED')

    @turnstile_locked.event
    def on_msg(agent, message):
        """COIN: Change state to unlocked. PUSH: Print denial message."""
        if message == 'COIN':
            print('Coin received, unlocking!')
            agent.statemachine.change_state(turnstile_unlocked)
        if message == 'PUSH':
            print('Denied!')

    turnstile_unlocked = State('STATE_UNLOCKED')
    @turnstile_unlocked.event
    def on_msg(agent, message):
        if message == 'PUSH':
            print('There it goes!')
            agent.statemachine.change_state(turnstile_locked)

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