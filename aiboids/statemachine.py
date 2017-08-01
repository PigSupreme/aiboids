#!/usr/bin/env python
"""Module containing basic UML State Machine functionality.

Actual states should be instances of the State class; its documentation shows
how to define the necessary state logic methods. We provide STATE_NONE as a
simple example; this can also be used for an concrete initial state.

The object `agent` can be given state machine functionality as follows::

    fsm = StateMachine(agent, cur=cur_state, glo=glo_state, pre=pre_state)

The keyword arguments are option. This als sets `agent.statemachine` = fsm
for later convenience. Alternatively, use Statemachine.set_state() later.

Once the state machine is initiliazed, state logic and messaging are done with
the respective functions::
    
    agent.statemachine.update()
    agent.statemachine.handle_msg(message)
    
One can, of course, call these methods directly from the StateMachine instance.
"""

from __future__ import print_function

class State(object):
    """Provides the necessary framework for UML states with trigger events.

    Args:
        name (string): The name of this state; see note below.

    Instances of this class are initiated with a set of default hooks for the
    state machine logic: on_enter, on_execute, on_exit, and on_msg.

    The defaults do nothing, but can be overridden on a per-instance basis.
    Each enter/execute/exit method must be a callable with a single positional
    parameter: the agent using this State's logic. The on_msg method needs an
    additional second positional parameter to hold the incoming message.

    After calling statename = State(...), the decorator @statename.event is
    available to easily replace the default hook methods above. For example::

        STATE_NONE = State('STATE_NONE')
        @STATE_NONE.event
        def on_enter(agent):
            print("WARNING: Agent %s entered STATE_NONE" % agent)

    This will set STATE_NONE.on_enter to the newly-defined function, and
    automatically print some documentation; see the output of sample_run().
    When overriding multiple hooks for the same state, the decorator must
    appear before each new function definition.

    Note:
        For future compatibility, instantiate each state as a CONSTANT_VALUE
        with a name that matches, as in the example.
    """
    def __init__(self, name):
        self.on_enter = State.on_dummy
        self.on_execute = State.on_dummy
        self.on_exit = State.on_dummy
        self.on_msg = State.msg_dummy
        self.name = str(name)
        self.register_state(self, name)

    @classmethod
    # For keeping track of defined states
    def register_state(cls, state, statename):
        # TODO: Called internally whenever a new State() is instantiated.
        # TODO: In the future, we'll use this...somehow.
        if not hasattr(cls, 'statedir'):
            cls.statedir = {}
        cls.statedir[statename] = state

    # Dummy placeholder for on{enter/execute/exit} methods.
    @staticmethod
    def on_dummy(dummy_agent):
        pass

    # Dummy placeholder for on_msg.
    @staticmethod
    def msg_dummy(dummy_agent, dummy_msg):
        return False

    def event(self, eventhook):
        # This defines the @state.event decorator as described above.
        if eventhook.__name__ in ('on_enter','on_execute','on_exit','on_msg'):
            setattr(self, eventhook.__name__, eventhook)
            # Note how we can grab the docstring using the decorator!
            if eventhook.__doc__:
                print("%s.%s() docstring:" % (str(self.name), eventhook.__name__))
                print(" %s\n" % eventhook.__doc__)
                # TODO: We can change the docstring, but need to figure out how autodocs can find it.
                # setattr(eventhook, "__doc__", "GOAT!")
            # This return value ensures that an Exception is raised if we try
            # to use the decorated function from outside of the given State.
            # TODO: Instead of print()ing, store for later in a class variable/method? 
            return None
        else:
            raise ValueError('State event %s not recognized' % eventhook.__name__)

################ A very simple State #####################
#: Used as a default null state by the StateMachine class.
STATE_NONE = State('STATE_NONE')

@STATE_NONE.event
def on_enter(agent):
    """Dummy docstring for discovery by @state.event decorator."""
    print("WARNING: Agent %s entered STATE_NONE" % agent)
##########################################################

class StateMachine(object):
    """UML-Style State Machine with event/messaging capability.

    Args:
        agent (object): The agent that will use this StateMachine.

    Keyword Args:
        cur (State): Sets the current state.
        glo (State): Sets the global state.
        pre (State): Sets the previous state.

    Instantiating this class will automatically set agent.statemachine to the
    new instance. Omitting any of the keyword arguments will set their
    corresponding states to None; these can also be assigned/changed later
    using StateMachine.set_state().
    """

    def __init__(self, agent, **kwargs):
        self.agent = agent
        agent.statemachine = self
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
        """Manually set agent's states without triggering state change logic.

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
        """Call on_enter() for global, then current state."""
        if self.glo_state:
            self.glo_state.on_enter(self.agent)
        if self.cur_state:
            self.cur_state.on_enter(self.agent)

    def update(self):
        """Call on_execute() for global, then current state."""
        # First execute a global state if it exists
        if self.glo_state:
            self.glo_state.on_execute(self.agent)
        # Now execute the current state (which should exist, but check anyway)
        if self.cur_state:
            self.cur_state.on_execute(self.agent)

    def shutdown(self):
        """Call on_exit() for global, then current state."""
        if self.glo_state:
            self.glo_state.on_exit(self.agent)
        if self.cur_state:
            self.cur_state.on_exit(self.agent)

    def change_state(self, newstate):
        """Switches to a new state, calling appropriate exit/enter methods.

        Args:
            newstate (State): The StateMachine will switch to this state.

        Assuming the current and newstates are both valid, this does:

        * Call on_exit() of the current state.

        * Change to the newstate.

        * Call on_enter() of the newstate.

        """
        if self.cur_state and newstate:
            self.pre_state = self.cur_state
            self.cur_state.on_exit(self.agent)
            self.cur_state = newstate
            self.cur_state.on_enter(self.agent)

    def revert_state(self):
        """Reverts agent to its previous state, if one exists."""
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
        if self.cur_state and self.cur_state.on_msg(self.agent, message):
            return True
        # ...but if handling was not done, send to the global state.
        if self.glo_state and self.glo_state.on_msg(self.agent, message):
            return True
        else:
            return False

def sample_run():
    """Demo based on the classic two-state turnstile FSM."""
    class Turnstile(object):
        """A virtual turnstile with counter."""

        def __init__(self):
            self.counter = 0

### TURNSTILE_LOCKED State Logic ####################################
    TURNSTILE_LOCKED = State('TURNSTILE_LOCKED')
    @TURNSTILE_LOCKED.event
    def on_msg(agent, message):
        """Acts on the following messages:
        * 'COIN': Change state to TURNSTILE_UNLOCKED.
        * 'PUSH': Access denied; print some text.
        """
        if message == 'COIN':
            print('Coin inserted!')
            agent.statemachine.change_state(TURNSTILE_UNLOCKED)
        if message == 'PUSH':
            print('Denied!')
#####################################################################

### TURNSTILE_UNLOCKED State Logic ##################################
    TURNSTILE_UNLOCKED = State('TURNSTILE_UNLOCKED')
    @TURNSTILE_UNLOCKED.event
    def on_msg(agent, message):
        """Acts on the following messages:
        * 'PUSH': Change state to TURNSTILE_LOCKED.
        """
        if message == 'PUSH':
            print('There it goes!')
            agent.statemachine.change_state(TURNSTILE_LOCKED)
    @TURNSTILE_UNLOCKED.event
    def on_exit(agent):
        """Increment turnstile counter.
        """
        # Not the best place for this, but an excuse to demo on_exit().
        agent.counter += 1
#####################################################################

    # Initialize the Turnstile object and state machine
    thing = Turnstile()
    fsm = StateMachine(thing)
    fsm.set_state(TURNSTILE_LOCKED, glo=STATE_NONE)
    fsm.start()

    # Main loop
    for i in range(1,20):
        print('\nUpdate %d:' % i)
        if i%5 == 0:
            print('Inserting coin...')
            fsm.handle_msg('COIN')

        if i%3 == 0:
            print('Trying to push...')
            fsm.handle_msg('PUSH')
        fsm.update()

    # Print final stats
    print("Final passthrough count: %d" % thing.counter)

if __name__ == "__main__":
    sample_run()
