"""A local process lease covering camera opens and USB rebind gaps.

All entry points controlling the same camera must use the same lock path.
The path is supplied by the application: this module has no robot, service
manager, shutdown marker or USB dependency. The lock is advisory and its file
must remain in place; removing it could let another process lock a new inode.
"""

from contextlib import contextmanager
import fcntl
import os
from pathlib import Path
import stat


class ProcessOwnershipError(RuntimeError):
    """Local exclusive ownership could not be established."""


class OwnershipBusyError(ProcessOwnershipError):
    """Another local process owns this camera lease."""


class InvalidOwnershipFileError(ProcessOwnershipError):
    """The lock path is not a single, stable regular file."""


@contextmanager
def process_ownership(path):
    """Hold a nonblocking exclusive lease until normal or exceptional exit.

    The parent directory must already exist. No camera is opened or probed,
    and a busy owner is never interrupted. Kernel close releases the lease
    even when its process exits without running Python cleanup.
    """
    path = Path(path)
    descriptor = os.open(path, os.O_RDONLY | os.O_CREAT | os.O_NOFOLLOW | os.O_CLOEXEC, 0o644)
    try:
        info = os.fstat(descriptor)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            raise InvalidOwnershipFileError('Camera ownership path must be a single regular file')
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise OwnershipBusyError('Camera is already owned by another local process') from error
        # Reject replacement during open/lock acquisition. Applications must
        # not unlink this file later: flock protects its inode, not its name.
        current = path.stat(follow_symlinks=False)
        if (not stat.S_ISREG(current.st_mode) or current.st_nlink != 1
                or (current.st_dev, current.st_ino) != (info.st_dev, info.st_ino)):
            raise InvalidOwnershipFileError('Camera ownership path changed during acquisition')
        yield
    finally:
        os.close(descriptor)
