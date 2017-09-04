#!/usr/bin/env python
"""Module for defining and managing game-type entities.

Use subclasses of BaseEntity for agents that need a unique identifier,
periodic updates, and messaging functionality. Subclasses need to:

* Implement update(self); for the general update cycle.

* Implement receive_msg(self, message); to enable messaging.

* Call BaseEntity.__init__() during the subclass __init__() method.

BaseEntity uses several static methods for manger-type functionality; these
are called on BaseEntity and NOT on subclasses. Most commonly-needed are::

    BaseEntity.by_id(id_string) # To get an entity from its identifier.
    BaseEntity.update_all()     # Call entity.update() on every entity.
    BaseEntity.call_on_all(f, *a, **k)  # Call entity.f(*a, **k) on every entity.

The PostOffice class is used for message handling; only one instance should be
needed. Instantiating a PostOffice() will afterwards allow an entity to use::

    entity.send_msg(recv, tag, extra, **kwargs)

without needing to call the PostOffice explicitly. It is possible [but not yet
tested] to use multiple instances of PostOffice, in which case only the most
recently instantiated will have this automatic functionality. See send_msg()
for further details and available keyword arguments.

To send a messages directly, without using a BaseEntity::

    po = PostOffice(...)
    po.post_msg(send, recv, tag, extra, **kwargs)

The documenation for PostOffice.post_msg() has a complete explanation of usage
and keyword arguments.
"""

# for python3 compat
#from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import print_function

from collections import namedtuple  # For EntityMessage structure
import heapq  # For PostOffice message queue

class BaseEntity(object):
    """Abstract Base Class for objects with an ID, update, and messaging.

    Args:
        id_string (string): An identifier for this entity; must be unique.

    Raises:
        ValueError: If id_string has already been used by some entity.

    Example:
        >>> goat = BaseEntity('THIS_GOAT')
        >>> another_goat = BaseEntity('THIS_GOAT')
        Traceback (most recent call last):
        ValueError: EntityID THIS_GOAT has already been used.
        >>> goat.update()
        Traceback (most recent call last):
        NotImplementedError: THIS_GOAT of <class '__main__.BaseEntity'> has undefined update().
        >>> goat.receive_msg('Hey, goat!')
        Traceback (most recent call last):
        NotImplementedError: THIS_GOAT of <class '__main__.BaseEntity'> has undefined receive_msg().

    Todo:
        Figure out a useful implemenation for invalid entities that will work
        with deleting/removing entities from the registry.
    """

    # TODO: Implement this better.
    _INVALID_ID = 'INVALID_ENTITY'
    _registry = {_INVALID_ID: None}

    def __init__(self, id_string):
        if id_string in BaseEntity._registry:
            raise ValueError('EntityID %s has already been used.' % id_string)
        else:
            BaseEntity._registry[id_string] = self
            self._mySTR = id_string
            self._myID = id(self)

    def update(self):
        """Generic update hook; must be implemented by subclasses/instances."""
        raise NotImplementedError("%s of %s has undefined update()." % (self.idstr, self.__class__))

    def receive_msg(self, message):
        """Generic message hook; must be implemented by subclasses/instances."""
        raise NotImplementedError("%s of %s has undefined receive_msg()." % (self.idstr, self.__class__))

    @property
    def idstr(self):
        """Get the user-supplied ID string of this entity.

        >>> ent = BaseEntity('SOME_PIG')
        >>> print(ent.idstr)
        SOME_PIG
        """
        return self._mySTR

    @staticmethod
    def by_id(id_str):
        """Get an entity from its ID string; use BaseEntity.by_id().

        >>> cmnsm = BaseEntity('ANY_FISH')
        >>> cmnsm = BaseEntity('RED_HERRING')
        >>> cmnsm == BaseEntity.by_id('ANY_FISH')
        False
        >>> cmnsm == BaseEntity.by_id('RED_HERRING')
        True
        >>> print(BaseEntity.by_id('NO_SUCH_ENTITY'))
        None

        Returns:
            The given entity, or None if no such ID is registered.
        """
        try:
            result = BaseEntity._registry[id_str]
        except KeyError:
            result = None
        return result

    @staticmethod
    def update_all():
        """Call the update() method of every registered entity.

        Syntax for this is `BaseEntity.update_all()`.
        """
        for entity in BaseEntity._registry.values():
        #TODO: try..except is needed only for the default entry in _registry
        #TODO: We possibly need better error-checking
            try:
                entity.update()
            except AttributeError:
                if entity is not None:
                    raise

    @staticmethod
    def call_on_all(func, *args, **kwargs):
        """Call a function on every registered entity.

        >>> entlist = []
        >>> getid  = lambda entity: entlist.append(entity.idstr)
        >>> BaseEntity.call_on_all(getid)
        >>> entlist.sort()
        >>> entlist
        ['ANY_FISH', 'RED_HERRING', 'THIS_GOAT']

        Note:
            doctest.testmod() seems to be running tests in alphabetic order,
            so BaseEnitty.idstr hasn't been tested, and SOME_PIG is missing.
        """
        for entity in BaseEntity._registry.values():
            try:
                func(entity, *args, **kwargs)
            except AttributeError:
                if entity is not None:
                    raise

    # Note: This function will be picked up by autodocs, but is overridden
    # automatically when a PostOffice() is instantiated.
    def send_msg(self, recv_id, msg_tag, extra=None, **kwargs):
        """Send a message now or add to the queue for later delivery.

        Args:
            recv_id: ID of the recipient, compatible with BaseEntity.by_id().
            msg_tag (string): A general tag/type for the message.
            extra: Optional message information.

        Keyword Args:
            at_time (float): Deliver at this absolute time. If given, `delay`
                is ignored.
            delay (float): Deliver after this amount of time.

        This is a convenience function that allows entities to send messages
        without explicitly calling a PostOffice, altough the PostOffice must
        already be instantiated. See PostOffice.post_msg() for a complete
        explanation of the available arguments.

        Raises:
            RuntimeError: If a PostOffice has not yet been instantiated.
        """
        raise RuntimeError("Entity messaging unavailable; a PostOffice does not exist.")

    # This version becomes available after a PostOffice() is instantiated, but
    # won't be picked up by autodocs (unless private members are enabled).
    @staticmethod
    def _set_postoffice(postoffice):
        """Called by PostOffice.__init__() to set up messaging functions."""
        BaseEntity.postoffice = postoffice

        def send_msg(self, recv_id, msg_tag, extra=None, **kwargs):
            """Can be used by entities to send messages; see below for usage.

            Args:
                recv_id: ID of the recipient, compatible with BaseEntity.by_id().
                msg_tag (string): A general tag/type for the message; see below.
                extra: Optional message information; see below.

            Once a PostOffice has been initialized, entity.send_msg() is available;
            this eliminates the need for each entity to keep track of the PostOffice.
            See PostOffice.post_msg() docs for additional usage information.
            """
        # TODO: Use functools.partial to simplify this and reduce function call overhead.
        # TODO: Additional functionality/syntax for sending messages to oneself?
            postoffice.post_msg(self.idstr, recv_id, msg_tag, extra, **kwargs)
        BaseEntity.send_msg = send_msg

class PostOffice(object):
    """Class for posting/handling messages between entities.

    Args:
        clock: An object with a time() method (callable with no parameters).

    The PostOffice uses its assigned `clock` in order to manage timestamps and
    the delivery schedule. A very basic clock class, DummyClock, is provided
    for demonstration purposes. If using any kind of delayed messages, be sure
    to periodically update the clock and call PostOffice.dispatch_queued().

    See the post_msg() method for full details on sending messages.

    Whenever a PostOffice() is instantiated, the BaseEntity.send_msg() method
    (described above) is automatically updated to use the new PostOffice().
    """
    class EntityMessage(namedtuple('Message', 'MSG_TYPE, TIMESTAMP, SEND_ID, RECV_ID, EXTRA')):
        """An envelope/message format; internal use only."""

    def __init__(self, clock):
        self.message_q = [] # Internally, this uses a heap.
        self.clock = clock  # For time-keeping, must have a time() method.
        self.lookup = BaseEntity.by_id # For entity lookup
        BaseEntity._set_postoffice(self)

    def post_msg(self, send_id, recv_id, msg_tag, extra=None, **kwargs):
        """Send a message now or add to the queue for later delivery.

        Args:
            send_id: ID of the sender, compatible with BaseEntity.by_id().
            recv_id: ID of the recipient, compatible with BaseEntity.by_id().
            msg_tag (string): A general tag/type for the message; see below.
            extra: Optional message information; see below.

        Keyword Args:
            at_time (float): Deliver at this absolute time. If given, `delay`
                is ignored; see below.
            delay (float): Deliver after this amount of time; see below.

        The `msg_tag` is intended for simple messages; in some cases, this is
        sufficient. For more complex messaging needs, the tag can be used as a
        category, and more specific information can be included using `extra`.
        In any case, the receiver has sole responsibility for dealing with any
        messages; it may even choose to ignore them completely.

        Messages with the `at_time` or `delay` keyword (the first supercedes
        the second) are scheduled based on the PostOffice's clock.time(). If
        neither option is given or the scheduled time is computed to be in the
        past, the message gets delivered immediately. The dispatch_queued()
        function must be called periodically in order for delayed messages to
        be processed and delivered.

        For now, any message that fails to deliver is just discarded, but this
        behaviour may change in the future.
        """
        # If recipient is invalid, exit immediately
        receiver = self.lookup(recv_id)
        if not receiver:
            return None

        # Compute the timestamp using kwargs
        time_now = self.clock.time()
        if 'at_time' in kwargs.keys():
            timestamp = kwargs['at_time']
        elif 'delay' in kwargs.keys():
            timestamp = time_now + kwargs['delay']
        else:   # Neither keyword was given; deliver immediately.
            timestamp = time_now

        # Create the actual message object
        message = PostOffice.EntityMessage(msg_tag, timestamp, send_id, recv_id, extra)
        if timestamp <= time_now:
            # Discharge immediately
            receiver.receive_msg(message)
        else:
            # Add this message to the delayed queue
            heapq.heappush(self.message_q, (timestamp, message))

    def dispatch_queued(self):
        """Checks for and dispatches messages from the delayed message queue."""
        # Dispatch messages until queue empty or next message in the future
        time_now = self.clock.time()
        while len(self.message_q) > 0 and self.message_q[0][0] <= time_now:
            msg = heapq.heappop(self.message_q)[1]
            receiver = self.lookup(msg.RECV_ID)
            if receiver:
                receiver.receive_msg(msg)

class DummyClock(object):
    """A clock-like object for a simulated discrete timeline.

    When initiated, clock time is set to 0. Call the tick() method to increase
    the time by 1, thus time() will always be a non-negative integer.
    """
    def __init__(self):
        self.now = 0
    def tick(self):
        self.now += 1
    def time(self):
        return self.now

def sample_run():
    """Demo showing the various types of messaging."""
    class SampleEnt(BaseEntity):
        def update(self):
            print("(%s): Nothing to see here..." % self.idstr)
        def receive_msg(self, message):
            print("(%s) got message: %s" % (self.idstr, message))

    # Initialization
    mc = DummyClock()
    po = PostOffice(mc)
    this = SampleEnt('THIS_ENT')
    that = SampleEnt('THAT_ENT')

    # Time = 1
    mc.tick()
    print("# Time is now %d" % mc.time())
    BaseEntity.update_all()
    this.send_msg('THIS_ENT', 'IMMEDIATE')
    po.dispatch_queued()

    # Time = 2
    mc.tick()
    print("# Time is now %d" % mc.time())
    BaseEntity.update_all()
    this.send_msg('THAT_ENT', 'RELATIVE_TIME', delay=2)
    that.send_msg('THIS_ENT', 'ABSOLUTE_TIME', at_time=2)

    # Time = 3
    mc.tick()
    print("# Time is now %d" % mc.time())
    BaseEntity.update_all()
    po.post_msg('THAT_ENT', 'THIS_ENT', 'VIA_POSTOFFICE', delay=1)
    po.dispatch_queued()

    # Time = 4
    mc.tick()
    print("# Time is now %d" % mc.time())
    BaseEntity.update_all()
    po.dispatch_queued()

if __name__ == "__main__":
    # TODO: Doctests need some tweaking due to order of execution.
    sample_run()
