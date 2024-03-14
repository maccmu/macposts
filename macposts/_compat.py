"""Compatibility module for macposts.

This module provides aliases and obsolete definitions to avoid breakage.

"""

__all__ = [
    "Dta_Api",
    "dta_api",
    "Mcdta_Api",
    "mcdta_api",
    "tdsp_api",
    "mmdta_api",
]

from _macposts_ext import (
    Dta as Dta_Api,
    Mcdta as Mcdta_Api,
    Tdsp as tdsp_api,
    Mmdta as mmdta_api,
)

dta_api = Dta_Api
mcdta_api = Mcdta_Api
