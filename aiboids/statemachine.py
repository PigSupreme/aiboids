#!/usr/bin/env python
"""Module containing basic UML State Machine functionality.

----

Actual states should be instances of the State class. We provide STATE_NONE as
a simple example; this can also be used for an concrete initial state.

Instances of the State class are initiated with a set of default hooks for the
state machine logic: on_enter(), on_execute(), on_exit(), and on_msg. These do
nothing unless overridden as follows:

After calling statename = State(...), the decorator @statename.event is used
to replace the default hooks. For example::

    STATE_NONE = State('STATE_NONE')
    @STATE_NONE.event
    def on_enter(agent):
        # This will replace STATE_NONE.on_enter()
        print("WARNING: Agent %s entered STATE_NONE" % agent)

Each enter/execute/exit method must be a callable with a single positional
parameter: the *agent* using this State's logic.

An on_msg method needs an additional second positional parameter to hold the
incoming message. It should True/False to indicate whether the message was
succesfully handled; messages that were not handled by the current state are
then sent to the global state.

Decorating a function in this way...

----

The Python object `agent` can be given state machine functionality as follows::

    StateMachine(agent, cur=cur_state, glo=glo_state, pre=pre_state)

Keyword arguments are optional; Statemachine.set_state() can be used to set
states at a later time. This sets `agent.statemachine` to the newly created
StateMachine instance, thus state logic and messaging can be done with the
respective function calls::

    agent.statemachine.update()
    agent.statemachine.handle_msg(message)

----
"""

class State(object):
    """Provides the necessary framework for UML states with trigger events.

    Args:
        name (string): The name of this state.

    It is recommended that each instance is a CONSTANT_VALUE with a matching
    name, such as::

        STATE_NONE = State('STATE_NONE')

    See the module docstring for full usage and examples.
    """
    def __init__(self, name):
        self.on_enter = State._on_dummy
        self.on_execute = State._on_dummy
        self.on_exit = State._on_dummy
        self.on_msg = State._msg_dummy
        self.name = str(name)
        self.register_state(self, name)

    @staticmethod
    def register_state(state, statename):
        # Called internally when a new state is instantiated.
        # Keeps track of defined states
        try:
            State.statedir[statename] = state
        except AttributeError:
            State.statedir = dict()
            State.statedir[statename] = state

    # Dummy placeholder for on{enter/execute/exit} methods.
    @staticmethod
    def _on_dummy(dummy_agent):
        pass

    # Dummy placeholder for on_msg.
    @staticmethod
    def _msg_dummy(dummy_agent, dummy_msg):
        return False

    def event(self, eventhook):
        """Decorator for assigning event functions to this State."""
        if eventhook.__name__ in ('on_enter', 'on_execute', 'on_exit', 'on_msg'):
            setattr(self, eventhook.__name__, eventhook)
            # This return value ensures that an Exception is raised if we try
            # to use the decorated function from outside of the given State.
            return None
        else:
            raise ValueError('State event %s not recognized' % eventhook.__name__)

    @staticmethod
    def print_state_events():
        """Lists all defined state events/docstrings; mostly for debugging."""
        print('\n'+33*'=')
        for name, state in State.statedir.items():
            print('--- %s defined events ---' % name)
            for eventhook in ('on_enter', 'on_execute', 'on_exit'):
                if state.__getattribute__(eventhook) != State._on_dummy:
                    print('%s: %s()' % (name, eventhook))
                    print('   ', state.__getattribute__(eventhook).__doc__, '\n')
            if state.on_msg != State._msg_dummy:
                print(name, ': on_msg()')
                print('   ', state.on_msg.__doc__, '\n')

################ A very simple State #####################
#: Used as a default null state by the StateMachine class.
STATE_NONE = State('STATE_NONE')

@STATE_NONE.event
def on_enter(agent):
    """Dummy docstring for discovery by @state.event decorator."""
    print("Agent %s entered STATE_NONE" % agent)
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
    corresponding states to STATE_NONE; these can be assigned/changed later
    using StateMachine.set_state().
    """

    def __init__(self, agent, **kwargs):
        self.agent = agent
        agent.statemachine = self
        for stype in ('cur', 'glo', 'pre'):
            if stype in kwargs:
                setattr(self, stype+'_state', kwargs[stype])
            else:
                setattr(self, stype+'_state', STATE_NONE)

    @property
    def statename(self):
        """Get the name of this state machine's current state."""
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
        """Calls on_enter() for global, then current state."""
        if self.glo_state:
            self.glo_state.on_enter(self.agent)
        if self.cur_state:
            self.cur_state.on_enter(self.agent)

    def update(self):
        """Calls on_execute() for global, then current state."""
        # First execute a global state if it exists
        if self.glo_state:
            self.glo_state.on_execute(self.agent)
        # Now execute the current state (which should exist, but check anyway)
        if self.cur_state:
            self.cur_state.on_execute(self.agent)

    def shutdown(self):
        """Calls on_exit() for global, then current state."""
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
        return self.glo_state and self.glo_state.on_msg(self.agent, message)

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
        """Increment turnstile counter."""
        # Not the best place for this, but an excuse to demo on_exit().
        agent.counter += 1
#####################################################################

    State.print_state_events()

    # Initialize the Turnstile object and state machine
    thing = Turnstile()
    fsm = StateMachine(thing)
    fsm.set_state(TURNSTILE_LOCKED, glo=STATE_NONE)
    fsm.start()

    # Main loop
    for i in range(1, 20):
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
