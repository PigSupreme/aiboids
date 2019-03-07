#!/usr/bin/env python3
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

from collections import namedtuple  # For EntityMessage structure
import sched  # For automatic scheduling of delayed messages

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
        NotImplementedError: THIS_GOAT of <class 'aiboids.base_entity.BaseEntity'> has undefined update().
        >>> goat.receive_msg('Hey, goat!')
        Traceback (most recent call last):
        NotImplementedError: THIS_GOAT of <class 'aiboids.base_entity.BaseEntity'> has undefined receive_msg().
    """

    _INVALID_ID = 'INVALID_ENTITY'
    _registry = dict()

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

        Example:
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
            if result == BaseEntity._INVALID_ID:
                return None
        except KeyError:
            result = None
        return result

    @staticmethod
    def update_all():
        """Call the update() method of every registered entity.

        Syntax for this is `BaseEntity.update_all()`.
        """
        for entity in BaseEntity._registry.values():
            if entity != BaseEntity._INVALID_ID:
                entity.update()

    @staticmethod
    def call_on_all(func, *args, **kwargs):
        """Call a function on every registered entity.

        >>> entlist = []
        >>> getid  = lambda entity: entlist.append(entity.idstr)
        >>> BaseEntity.call_on_all(getid)
        >>> sorted(entlist)
        ['ANY_FISH', 'RED_HERRING', 'THIS_GOAT']
        """
        for entity in BaseEntity._registry.values():
            if entity != BaseEntity._INVALID_ID:
                func(entity, *args, **kwargs)

    @staticmethod
    def remove(id_str, delete=True):
        """Remove an entity's from the registry using its ID String.

        Args:
            id_str (string): The ID string of the entity.
            delete (boolean): If True (default), the entity with this ID is
                deleted after unregistering.

        Note:
            To avoid reference issues, an ID string that gets removed will not
            be available for later use

        >>> BaseEntity.remove('RED_HERRING')
        >>> print(BaseEntity.by_id('RED_HERRING'))
        None
        >>> BaseEntity('RED_HERRING')
        Traceback (most recent call last):
        ValueError: EntityID RED_HERRING has already been used.
        """
        entity = BaseEntity.by_id(id_str)
        if entity:
            BaseEntity._registry[id_str] = BaseEntity._INVALID_ID
            if delete:
                del entity


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
        """Called by PostOffice.__init__() to set up messaging functions.

        When a PostOffice is initialized, this function is called to update the
        BaseEntity.send_msg() method, eliminating the need for each entity to
        keep track of the PostOffice.
        """
        def send_msg(self, recv_id, msg_tag, extra=None, **kwargs):
            """Replaces BaseEntity.send_msg to use the new PostOffice."""
            postoffice.post_msg(self.idstr, recv_id, msg_tag, extra, **kwargs)
        BaseEntity.postoffice = postoffice
        BaseEntity.send_msg = send_msg

class PostOffice(object):
    """Class for posting/handling messages between entities.

    Args:
        scheduler (sched.scheduler): See below.

    The PostOffice needs an external scheduler (from sched.py) to manage
    timestamps and handle delivery of delayed messages. Whenever a PostOffice()
    is instantiated, the BaseEntity.send_msg() method (described above) is
    automatically updated to use the new PostOffice().

    See the post_msg() method below for full details on sending messages.
    """
    class EntityMessage(namedtuple('Message', 'MSG_TYPE, TIMESTAMP, SEND_ID, RECV_ID, EXTRA')):
        """An envelope/message format; internal use only."""

    def __init__(self, scheduler):
        self.scheduler = scheduler
        self.DELAY_PRIORITY = 2 # Schedule priority for delayed messages
        self.lookup = BaseEntity.by_id # For entity lookup
        BaseEntity._set_postoffice(self)

    def _deliver(self, message):
        """Used internally by the scheduler to deliver delayed messages."""
        receiver = self.lookup(message.RECV_ID)
        if receiver:
            receiver.receive_msg(message)

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

        The `msg_tag` is intended for simple messages; in many cases this is
        sufficient. For more complex messaging needs, the tag can be used as a
        category, and more specific information can be included using `extra`.
        In any case, the receiver has sole responsibility for dealing with any
        messages; it may even choose to ignore them completely.

        Messages with the `at_time` or `delay` keyword (the first supercedes
        the second) use the _scheduler_ to compute delivery times. If neither
        option is given or the scheduled time is in the past, the message gets
        delivered as soon as possible.

        For now, any message that fails to deliver is just discarded, but this
        behaviour may change in the future.
        """
        # If recipient is invalid, exit immediately
        receiver = self.lookup(recv_id)
        if not receiver:
            return None

        # Compute the timestamp using the external scheduler:
        time_now = self.scheduler.timefunc()
        if 'at_time' in kwargs.keys():
            timestamp = kwargs['at_time']
        elif 'delay' in kwargs.keys():
            timestamp = time_now + kwargs['delay']
        else:   # Neither keyword was given; deliver immediately.
            timestamp = time_now

        # Create the actual message object
        message = PostOffice.EntityMessage(msg_tag, timestamp, send_id, recv_id, extra)
        if timestamp <= time_now: # Discharge immediately
            receiver.receive_msg(message)
        else: # Add this message to the delayed queue and schedule delivery
            self.scheduler.enterabs(timestamp, self.DELAY_PRIORITY, self._deliver, (message,))

    def update(self):
        """Use to force update of the delayed message queue."""
        self.scheduler.run(False)

class DummyClock(object):
    """A clock-like object for a simulated timeline, starting at 0.0."""
    def __init__(self):
        self.now = 0.0

    def tick(self, increment=1.0):
        """Advance the clock by the given amount."""
        self.now += max(0.0, increment)

    def time(self):
        """The current clock time."""
        return self.now


def sample_run():
    """Demo showing the various types of messaging."""
    class SampleEnt(BaseEntity):
        """Example BaseEntity subclass for demo."""
        def update(self):
            print("(%s): Nothing to see here..." % self.idstr)
        def receive_msg(self, message):
            print("(%s) got message: %s" % (self.idstr, message))

    # Initialization
    mc = DummyClock()
    sch = sched.scheduler(mc.time, mc.tick)
    po = PostOffice(sch)
    this = SampleEnt('THIS_ENT')
    that = SampleEnt('THAT_ENT')

    MAXTIME = 5
    def auto_tick(clock):
        """Used to schedule/propagate automatic clock and entity updates."""
        print('*** Clock *** Time is now %d' % clock.time())
        if 1 + clock.time() <= MAXTIME:
            sch.enter(1, 0, auto_tick, (clock,))
            sch.enter(1, 0.5, BaseEntity.update_all)
        else:
            print('*** Clock *** This will be the last automatic update.')

    # Schedule some messages
    sch.enterabs(1, 1, this.send_msg, ('THIS_ENT', 'IMMEDIATE'))
    sch.enterabs(2, 1, this.send_msg, ('THAT_ENT', 'RELATIVE_TIME'), {'delay': 2.2})
    sch.enterabs(2, 1, that.send_msg, ('THIS_ENT', 'ABSOLUTE_TIME'), {'at_time': 3.3})
    sch.enterabs(3, 1, po.post_msg, ('THAT_ENT', 'THIS_ENT', 'VIA_POSTOFFICE'), {'delay': 6})
    sch.enterabs(4, 1, that.send_msg, ('THIS_ENT', 'WITH EXTRAS'), {'extra': "Hey stuff!"})

    # Start the clock and run the schedule
    auto_tick(mc)
    sch.run()

if __name__ == "__main__":
    sample_run()
