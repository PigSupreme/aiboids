#!/usr/bin/env python
"""Two-Dimensional Point/Vector class with common operations.

Point2d() vectors support standard vector arithmetic: +, - (unary and binary),
and componentwise test for (in)equality. Multiplication a*b is overloaded with
the regular dot product; vdot(a,b) is also available. Scalar multiplication is
done functionally, using `a.scm(k)` for the scalar k and vector a.

Many other operations are available; see below for details.

TODO: Better Exception handling for the various arithmetic operators; see the
code for __mul__ (dot product) for an example of what we should be doing.
"""

# for python3 compat
from __future__ import unicode_literals
from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

from math import sqrt, acos, cos, sin

class Point2d(object):
    """A two-dimensional vector of floats, defaulting to <0.0, 0.0>.

    Args:
        x (float): x-coordinate (defaults to 0.0).
        y (float): y-coordinate (defaults to 0.0).
    """

    def __init__(self, x=0.0, y=0.0):
        self.nt = (float(x), float(y))

    def __getitem__(self, index):
        """Vector components; indexed starting at 0. For internal use only.

        >>> a = Point2d(1,-2)
        >>> [a[0], a[1]]   # For doctest only; use the syntax below instead.
        [1.0, -2.0]
        >>> [a.x, a.y]     # Preferred usage; same result.
        [1.0, -2.0]
        >>> a.ntuple       # Use this to get the vector as a python tuple.
        (1.0, -2.0)
        """
        return self.nt[index]

    @property
    def x(self):
        """Get the first (x) coordinate of this point; see example above."""
        return self.nt[0]

    @property
    def y(self):
        """Get the second (y) coordinate of this point; see example above."""
        return self.nt[1]

    @property
    def ntuple(self):
        """Get this point as a Python tuple; see example above."""
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
        self.nt = (0.0, 0.0)

    def __len__(self):
        # For internal use.
        return 2

    def __str__(self):
        return "Point2d: <%.9f, %.9f>" % self.nt

    def __eq__(self, other):
        """Test if two points are equal; overrides the == operator.

        >>> Point2d(1.0,0)==Point2d(1,0)
        True
        >>> Point2d(1,0)==Point2d(0,1)
        False
        >>> # When we override __eq__, we get inequality for free. Righteous!
        >>> Point2d(2,3) != Point2d(3,2)
        True
        """
        return self.nt[0] == other.nt[0] and self.nt[1] == other.nt[1]

    def __add__(self, term):
        """Coordinatewise addition; overrides the + operator.

        >>> a = Point2d(1,-2)
        >>> b = Point2d(3,5)
        >>> print(a+b)
        Point2d: <4.000000000, 3.000000000>
        >>> a += Point2d(1,1)
        >>> print(a)
        Point2d: <2.000000000, -1.000000000>
        """
        return Point2d(self.nt[0] + term.nt[0], self.nt[1] + term.nt[1])

    def __neg__(self):
        """Negates each entry; overrides unary - operator.

        >>> a = Point2d(1,-2)
        >>> print(-a)
        Point2d: <-1.000000000, 2.000000000>
        """
        return Point2d(-self.nt[0], -self.nt[1])

    def __sub__(self, term):
        """Coordinatewise subtraction; overrides the binary - operator.

        >>> a = Point2d(1,-2)
        >>> b = Point2d(3,5)
        >>> print(a-b)
        Point2d: <-2.000000000, -7.000000000>
        >>> a -= Point2d(1,1)
        >>> print(a)
        Point2d: <0.000000000, -3.000000000>
        """
        return Point2d(self.nt[0] - term.nt[0], self.nt[1] - term.nt[1])

    def __mul__(self, term):
        """Dot product: vector*vector.

        Examples below show the avaiable types of vector/scalar multiplcation.

        >>> a = Point2d(1,-2)
        >>> b = Point2d(3,5)
        >>> a*b         # Dot product as a binary operator
        -7.0
        >>> vdot(a,b)   # Dot product as a function of two vectors
        -7.0
        >>> print(3*a)  # Scalar multiplication, note the order
        Point2d: <3.000000000, -6.000000000>
        >>> a*3         # Vector*Scalar is undefined in this order
        Traceback (most recent call last):
        TypeError: Mutiplication of Point2d and <type 'int'> is undefined.
        >>> a *= a      # To avoid later confusion, this raises a TypeError.
        Traceback (most recent call last):
        TypeError: Compound assignment for dot product is not supported.
        """
        try:
            return (self.nt[0] * term.nt[0]) + (self.nt[1] * term.nt[1])
        except AttributeError:
            raise TypeError('Mutiplication of Point2d and %s is undefined.' % type(term))

    def __rmul__(self, scalar):
        """Scalar multiplication: scalar*vector; see examples above."""
        return Point2d(scalar*self.nt[0], scalar*self.nt[1])

    def __imul__(self, term):
        """Compound assignment is not supported for the dot product.

        Raises:
            TypeError
        """
        raise TypeError('Compound assignment for dot product is not supported.')

    def scm(self, scalar):
        """Scalar multiplication by this vector.

        Warning:
            Future deprecation is likely, use s*v instead of v.scm(s).

        >>> a = Point2d(1,-2)
        >>> print(a.scm(3.5))
        Point2d: <3.500000000, -7.000000000>
        >>> print(a.scm(-2))
        Point2d: <-2.000000000, 4.000000000>
        """
        return Point2d(scalar*self.nt[0], scalar*self.nt[1])

    def rotated_by(self, angle, use_deg=False):
        """Get this vector rotated anticlockwise.

        Args:
            angle (float): Directed anticlockwise angle in radians.
            use_deg (bool, optional): Set True to use degrees instead of radians.

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

        >>> Point2d(1,-2).norm()
        2.23606797749979
        """
        return sqrt(self.nt[0]**2 + self.nt[1]**2)

    def sqnorm(self):
        """Get the squared norm (length) of this vector.

        >>> Point2d(1,-2).sqnorm()
        5.0
        """
        return float(self.nt[0]**2 + self.nt[1]**2)

    def unit(self):
        """Get a unit vector in the same direction as this one.

        Raises:
            ZeroDivisionError: If called on a zero vector.

        Note:
            Be aware of round-off errors; see the example below.

        >>> a = Point2d(1,-2)
        >>> print(a.unit())
        Point2d: <0.447213595, -0.894427191>
        >>> a.unit().norm()
        0.9999999999999999
        """
        return self.scm(1.0/self.norm())

    def normalize(self):
        """Rescale this vector to have length 1.

        Raises:
            ZeroDivisionError: If called on a zero vector.

        Note:
            Be aware of round-off errors; see the example below.

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
        """Rescale this vector in-place to a maximum length.

        Args:
            maxlength (float): Upper limit on the length. If the current length
            exceeds this, the vector is rescaled in-place.

        Returns:
            (bool): True if rescaling was done, False otherwise.

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
        """Change this vector in-place to have the given magnitude.

        Args:
            mag (float): New magnitude; negative value also reverses direction.

        >>> a = Point2d(3,4)
        >>> a.scale_to(10)
        >>> print(a, a.norm())
        Point2d: <6.000000000, 8.000000000> 10.0
        >>> a.scale_to(-0.5)
        >>> print(a, a.norm())
        Point2d: <-0.300000000, -0.400000000> 0.5
        """
        mag = mag/self.norm()
        self.nt = (mag*self.nt[0], mag*self.nt[1])

    def angle(self):
        """Get the polar angle of this vector in radians; range (-pi,pi]

        Raises:
            ZeroDivisionError: If called on a zero vector.

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

        Args:
            direction (Point2d): The vector we project onto.

        Returns:
            float: The length of the orthogonal projection.

        Returns the scalar q such that self = q*u + r, where u is a unit
        vector (normalized direction) and r is orthogonal to u. This is
        algebraically identical to exact division (/). If you want the result
        as a vector, use Point2d.proj(direction) or compund assignment.

        Note:
            Compound assignment /= behaves differently; see below.

        >>> a = Point2d(2,2)
        >>> b = Point2d(3,0)
        >>> a/b
        2.0
        >>> b/a
        2.1213203435596424
        """
        # Note: * is the dot product, using __mul__ to override above.
        r = (self*direction)/direction.norm()
        return r

    def __itruediv__(self, direction):
        """In-place orthogonal projection; overrides the /= operator.

        Args:
            direction (Point2d): The vector we project onto.

        Replaces this vector by the unique vector v satisfying all of the
        following: (1) self = v + w, (2) v is parallel to direction, (3) w is
        orthogonal to v.

        Note:
            Non-compound division / behaves differently; see above.

        >>> a = Point2d(2,4)
        >>> b = Point2d(3,-2)
        >>> dotp = a*b
        >>> a /= b
        >>> print(a)
        Point2d: <-0.461538462, 0.307692308>
        >>> print(b)
        Point2d: <3.000000000, -2.000000000>
        >>> a*b == dotp  # Dot product is preserved by orthogonal projection.
        True
        """
        r = (self*direction)/direction.sqnorm()
        proj = direction.scm(r)
        return proj

    def proj(self, direction):
        """Get the orthogonal projection of this vector onto another.

        Args:
            direction (Point2d): The vector we project onto.

        Returns:
            Point2d: The unique vector v such that self = v + w, where v is
            parallel to direction and w is orthogonal to v.

        Note:
            If you want both v2 and v3, use Point2d.resolve(direction) instead.
            See divison / and compound divivion =/ above for other options.

        >>> a = Point2d(2,4)
        >>> b = Point2d(3,-2)
        >>> print(a.proj(b))
        Point2d: <-0.461538462, 0.307692308>
        >>> print(b.proj(a))
        Point2d: <-0.200000000, -0.400000000>
        """
        r = (self*direction)/direction.sqnorm()
        proj = direction.scm(r)
        return proj

    def resolve(self, direction):
        """Orthogonal decomposition of this vector in a given direction.

        Args:
            direction (Point2d): The vector we project onto.

        Returns:
            Point2d, Point2d: The unique vectors v, w such that self = v + w,
            where v is parallel to direction and w is orthogonal to v.

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

        >>> print(Point2d(1,-2).left_normal())
        Point2d: <2.000000000, 1.000000000>
        >>> print(Point2d(0,0).left_normal())
        Point2d: <-0.000000000, 0.000000000>

        Note:
            If applied to the zero vector, returns a zero vector (see example).
        """
        return Point2d(-self.nt[1], self.nt[0])

    def __setitem__(self, index, value):
        """Allows a value to be assigned to each vector components;
        indexed starting at 0.

        Note:
            Internal use only.

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

def vdot(v1, v2):
    """Dot product of two vectors; functional form.

    >>> Point2d(1,-2)*Point2d(3,5)           # As a binary operator
    -7.0
    >>> vdot(Point2d(1,-2), Point2d(3,5))    # As a function of two vectors.
    -7.0
    """
    return (v1[0] * v2[0]) + (v1[1] * v2[1])

if __name__ == "__main__":
    print("Point2d functions. Import this module elsewhere.")
    print("Running some doctests; if you see nothing past this, hooray!")
    import doctest
    doctest.testmod()
