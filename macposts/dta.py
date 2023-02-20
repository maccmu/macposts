"""Dynamic Traffic Assignment (DTA).

This module contains two classes for accessing the dynamic traffic assignment
functionalities in macposts. One is for single class DTA and the other is for
biclass DTA.

Note that this module is supposed to serve as an intermediate module between
the core `libmacposts' and the user.

"""

import _macposts_ext as _ext


# XXX: I would like to use a common base class instead.
class _CommonMixin:
    """Mixin class for common methods of both Dta and Mcdta."""

    @classmethod
    def from_files(cls, directory):
        """Create an instance of *cls* with files in *directory*."""
        obj = cls()
        obj.initialize(str(directory))
        return obj


class Dta(_CommonMixin, _ext.Dta):
    """Single class DTA."""


class Mcdta(_CommonMixin, _ext.Mcdta):
    """Biclass DTA."""
