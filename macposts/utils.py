"""Various utility functions.

This module provides some small utility functions which may be useful when
working with macposts.

"""

import os
import sys
import contextlib


try:
    STDOUT_FD = sys.__stdout__.fileno()
except Exception:
    STDOUT_FD = 1


@contextlib.contextmanager
def silence(fd=STDOUT_FD):
    """Suppress outputs to a file descriptor.

    Optional argument *fd* is the target and defaults to the file descriptor
    associated with stdout (in most cases it is 1). If it is not a valid file
    descriptor, nothing will be done.

    """
    try:
        os.fstat(fd)
    except Exception:
        try:
            yield
        finally:
            return

    save = os.dup(fd)
    null = os.open(os.devnull, os.O_RDWR)
    os.dup2(null, fd)
    try:
        yield
    finally:
        os.dup2(save, fd)
        os.close(null)
        os.close(save)
