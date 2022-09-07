"""Compatibility module for macposts.

This module provides aliases and obsolete definitions to avoid breakage.

"""

__all__ = [
    "Dta_Api",
    "Mcdta_Api"
]

from _macposts_ext import Dta as Dta_Api
from _macposts_ext import Mcdta as Mcdta_Api
