#!/usr/bin/env python
"""A Two-Dimensional Point/Vector Class.

In addition to the class methods documented below, the following operators
are available (but not picked up by autodocs):    
    
* a + b: Vector Addition
* -a : Vector Negation
* a - b: Vector Subtraction
* a * b: Dot Product (returns a scalar)
* a / b: Length of orthogonal project of a onto b.
* a == b: Test for equality
* a != b: Test for inequality
"""

# for python3 compat
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

from math import sqrt, acos, cos, sin

class Point2d(object):
    """Creates a 2d vector, defaulting to <0,0>.

    Parameters
    ----------
    x: float
        x-coordinate (defaults to 0).
    y: float
        y-coordinate (defaults to 0).
    """

    def __init__(self, x=0, y=0):
        self.nt = (float(x), float(y))

    @property
    def x(self):
        """The first (x) coordinate of this point."""
        return self.nt[0]
    
    @property
    def y(self):
        """The second (y) coordinate of this point."""
        return self.nt[1]

    @property
    def ntuple(self):
        """This point as a Python tuple."""
        return self.nt

    def zero(self):
        """Set all coordinates of this point to zero.

        >>> a = Point2d(3,-2)
        >>> print(a)
        Point2d: <3.000000000, -2.000000000>
        >>> a.zero()
        >>> print(a)
        Point2d: <0.000000000, 0.000000000>
        """
        self.nt = (0,0)

    def __len__(self):
        return 2

    def __str__(self):
        return "Point2d: <%.9f, %.9f>" % self.nt

    def __neg__(self):
        """Negates each entry; overrides unary - operator.

        Example
        -------
        >>> a = Point2d(1,-2)
        >>> print(-a)
        Point2d: <-1.000000000, 2.000000000>
        """
        return Point2d(-self.nt[0], -self.nt[1])

    def __eq__(self, other):
        """Test if two points are equal; overrides the == operator.

        Example
        -------
        >>> Point2d(1.0,0)==Point2d(1,0)
        True
        >>> Point2d(1,0)==Point2d(0,1)
        False
        >>> # When we override __eq__, we get inequality for free. Righteous!
        >>> Point2d(2,3)!=Point2d(3,2)
        True
        """
        return self.nt[0]==other.nt[0] and self.nt[1]==other.nt[1]

    def __add__(self, term):
        """Coordinatewise addition; overrides the + operator.

        Example
        -------
        >>> a = Point2d(1,-2)
        >>> b = Point2d(3,5)
        >>> print(a+b)
        Point2d: <4.000000000, 3.000000000>
        >>> a+=Point2d(1,1)
        >>> print(a)
        Point2d: <2.000000000, -1.000000000>
        """
        return Point2d(self.nt[0] + term.nt[0], self.nt[1] + term.nt[1])

    def __sub__(self, term):
        """Coordinatewise subtraction; overrides the - operator.

        Example
        -------
        >>> a = Point2d(1,-2)
        >>> b = Point2d(3,5)
        >>> print(a-b)
        Point2d: <-2.000000000, -7.000000000>
        >>> a-=Point2d(1,1)
        >>> print(a)
        Point2d: <0.000000000, -3.000000000>
        """
        return Point2d(self.nt[0] - term.nt[0], self.nt[1] - term.nt[1])

    def __mul__(self, term):
        """Dot product; overrides the \* operator.

        Example
        -------
        >>> Point2d(1,-2)*Point2d(3,5)
        -7.0
        >>> # But compound assignment *= should probably be avoided!
        >>> a = Point2d(1,2)  # This is a vector, for now...
        >>> a*=a              # ... but the dot product returns a scalar
        >>> a
        5.0
        """
        return (self.nt[0] * term.nt[0]) + (self.nt[1] * term.nt[1])

    def __getitem__(self, index):
        """Vector components; indexed starting at 0.

        Example
        -------
        >>> a = Point2d(1,-2)
        >>> a[0]
        1.0
        >>> a[1]
        -2.0
        """
        return self.nt[index]

    def scm(self, scalar):
        """Scalar multiplication of this vector.

        Example
        -------
        >>> a = Point2d(1,-2)
        >>> print(a.scm(3.5))
        Point2d: <3.500000000, -7.000000000>
        >>> print(a.scm(-2))
        Point2d: <-2.000000000, 4.000000000>
        """
        return Point2d(scalar*self.nt[0], scalar*self.nt[1])

    def rotated_by(self, angle, use_deg=False):
        """Get this vector rotated anticlockwise.

        Parameters
        ----------
        angle: int or float
            Directed anticlockwise angle to rotate by,
        degrees: boolean
            If True, angle is in degrees. Otherwise radians (default)

        Example
        -------
        >>> from math import pi
        >>> a = Point2d(2,-2)
        >>> print(a.rotated_by(pi))
        Point2d: <-2.000000000, 2.000000000>
        >>> print(a.rotated_by(-pi/3))
        Point2d: <-0.732050808, -2.732050808>
        >>> print(a.rotated_by(90,True))
        Point2d: <2.000000000, 2.000000000>
        """
        if use_deg is True:
            angle = angle / 57.2957795131

        c = cos(angle)
        s = sin(angle)
        return Point2d(c*self.nt[0] - s*self.nt[1], s*self.nt[0] + c*self.nt[1])

    def norm(self):
        """Get the norm (length) of this vector.

        Example
        -------
        >>> Point2d(1,-2).norm()
        2.23606797749979
        """
        return sqrt(self.nt[0]**2 + self.nt[1]**2)

    def sqnorm(self):
        """Get the squared norm (length) of this vector.

        Example
        -------
        >>> Point2d(1,-2).sqnorm()
        5.0
        """
        return float(self.nt[0]**2 + self.nt[1]**2)

    def unit(self):
        """Get a unit vector in the same direction as this one.

        Note
        ----
        Be aware of round-off errors; see the example below.

        Example
        -------
        >>> a = Point2d(1,-2)
        >>> print(a.unit())
        Point2d: <0.447213595, -0.894427191>
        >>> a.unit().norm()
        0.9999999999999999
        """
        return self.scm(1.0/self.norm())

    def normalize(self):
        """Rescale this vector to have length 1.

        Note
        ----
        Be aware of round-off errors; see the example below.

        Example
        -------
        >>> a = Point2d(1,-2)
        >>> a.normalize()
        >>> print(a)
        Point2d: <0.447213595, -0.894427191>
        >>> a.norm()
        0.9999999999999999
        """
        r = self.norm()
        self.nt = (float(self.nt[0]/r), float(self.nt[1]/r))

    def truncate(self, maxlength):
        """Rescale this vector if needed so its length is not too large.

        Parameters
        ----------
        maxlength: float
            Upper limit on the length. If the current length exceeds this,
            the vector will be rescaled.

        Returns
        -------
        bool:
            True if rescaling was done, False otherwise.

        Examples
        --------
        >>> a = Point2d(1,-2)
        >>> a.truncate(1.0)
        True
        >>> print(a)
        Point2d: <0.447213595, -0.894427191>
        >>> b = Point2d(-1,2)
        >>> b.truncate(5.0)
        False
        >>> print(b)
        Point2d: <-1.000000000, 2.000000000>
        """
        if self.sqnorm() > maxlength**2:
            r = float(maxlength/self.norm())
            self.nt = (self.nt[0]*r, self.nt[1]*r)
            return True
        else:
            return False

    def scale_to(self, mag):
        """Change this vector's scale to the given magnitude.

        Parameters
        ----------
        mag: float
            New magnitude for this vector; negative will reverse direction.

        Example
        -------
        >>> a = Point2d(2,3)
        >>> a.scale_to(-4.2)
        >>> print(a)
        Point2d: <-2.329740824, -3.494611236>
        >>> a.norm()
        4.2
        """
        self.normalize()
        self.nt = (mag*self.nt[0], mag*self.nt[1])

    def angle(self):
        """Get the polar angle of this vector in radians; range (-pi,pi]

        Raises
        ------
        ZeroDivisionError: If called on a zero vector.

        Examples
        --------
        >>> Point2d(1,-2).angle()
        -1.1071487177940904
        >>> Point2d(1,0).angle()
        0.0
        >>> Point2d(0,1).angle()
        1.5707963267948966
        >>> Point2d(-1,0).angle()
        3.141592653589793
        >>> Point2d(-2,-2).angle()
        -2.356194490192345
        >>> Point2d(0,0).angle()
        Traceback (most recent call last):
            ...
        ZeroDivisionError: float division by zero
        """
        theta = acos(self.nt[0]/self.norm())
        if self.nt[1] < 0:
            theta = -theta
        return float(theta)

    def __truediv__(self, direction):
        """Length of an orthogonal projection; overrides the / operator.

        Parameters
        ----------
        direction: Point2d
            The vector we project onto; not required to be a unit vector.

        Returns
        -------
        float:
            The length of the projection vector.

        Notes
        -----
        Returns the scalar q such that self = q*v2 + v3, where v2 is in the
        span of direction and v2 and v3 are orthogonal. This is algebraically
        identical to exact division (/).

        If you want the result as a vector, use Point2d.proj(direction) instead.

        Examples
        --------
        >>> a = Point2d(2,2)
        >>> b = Point2d(3,0)
        >>> a/b
        2.0
        >>> b/a
        2.1213203435596424
        >>> # Compound assignment /= is not recommended!
        >>> a=Point2d(1,2)
        >>> a/=a
        >>> a
        2.23606797749979
        """
        # Note: * is the dot product, using __mul__ to override above.
        r = (self*direction)/direction.norm()
        return r

    def proj(self, direction):
        """Get the orthogonal projection of this vector onto another.

        Parameters
        ----------
        direction: Point2d
            The vector we project onto; not required to be a unit vector.

        Returns
        -------
        Point2d
            The unique vector v2 such that self = q*v2 + v3, where v2 is in the
            span of direction and v2 and v3 are orthogonal.

        Example
        -------
        >>> a = Point2d(2,4)
        >>> b = Point2d(3,-2)
        >>> print(a.proj(b))
        Point2d: <-0.461538462, 0.307692308>
        >>> print(b.proj(a))
        Point2d: <-0.200000000, -0.400000000>

        Notes
        -----
        If you want both v2 and v3, use Point2d.resolve(direction) instead.
        """
        # Note: * is the dot product, using __mul__ to override above.
        r = (self*direction)/direction.sqnorm()
        proj = direction.scm(r)
        return proj

    def resolve(self, direction):
        """Orthogonal decomposition of this vector in a given direction.

        Parameters
        ----------
        direction: Point2d
            The vector we project onto; not required to be a unit vector.

        Returns
        -------
        Point2d, Point2d:
            v2,v3 such that self = q*v2 + v3, where v2 is in the
            span of direction and v2 and v3 are orthogonal.

        Example
        -------
        >>> a = Point2d(2,-3)
        >>> b = Point2d(1,4)
        >>> print(a.resolve(b)[0])
        Point2d: <-0.588235294, -2.352941176>
        >>> print(a.resolve(b)[1])
        Point2d: <2.588235294, -0.647058824>
        >>> print(a.resolve(b)[0]+a.resolve(b)[1])
        Point2d: <2.000000000, -3.000000000>
        """
        parallel = self.proj(direction)
        perp = self - parallel
        return parallel, perp

    def left_normal(self):
        """Returns the left-facing normal of this vector.

        Example
        -------
        >>> print(Point2d(1,-2).left_normal())
        Point2d: <2.000000000, 1.000000000>
        """
        return Point2d(-self.nt[1], self.nt[0])

    def __setitem__(self, index, value):
        """Allows a value to be assigned to each vector components;
        indexed starting at 0.

        Example
        -------
        >>> a = Point2d(1, -2)
        >>> print(a)
        Point2d: <1.000000000, -2.000000000>
        >>> a[0] = 3
        >>> print(a)
        Point2d: <3.000000000, -2.000000000>
        >>> a[1] = 5
        >>> print(a)
        Point2d: <3.000000000, 5.000000000>
        """
        if index == 0:
            self.nt = (value, self.nt[1])
        elif index == 1:
            self.nt = (self.nt[0], value)
        else:
            raise KeyError("Point2d %s has no component %s" % (self, str(index)))

if __name__ == "__main__":
    print("Point2d functions. Import this module elsewhere.")
    print("Running some doctests; if you see nothing past this, hooray!")
    import doctest
    doctest.testmod()
